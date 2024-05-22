#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_math_Additional.hpp>
#include <mpsv_math_Metric.hpp>
#include <mpsv_sampler_HaltonSequence.hpp>
#include <mpsv_geometry_OrientedBox.hpp>
#include <mpsv_core_DataLogFile.hpp>


namespace mpsv {


namespace core {


/**
 * @brief This class represents the base class for a node data type to be used by the FixedSizeTreeSE2.
 */
class FixedSizeTreeSE2Node {
    public:
        int16_t idxParent;                // Index of parent node (negative value if this is the root node).
        std::vector<int16_t> idxChilds;   // Indices of all child nodes.
        double cost;                      // The total cost value from the root node to this node.
        bool flag;                        // A custom flag. For example, it's used to mark all nodes to be removed. The actual meaning of this flag depends on the implementation that assignes and uses this flag.
        std::array<double, 3> pose;       // The 3-dimensional pose value of this node given as {x,y,psi}.

        /**
         * @brief Create a tree node.
         * @param[in] numChildsReserved Number of child nodes to reserve for this tree node.
         */
        explicit FixedSizeTreeSE2Node(const size_t numChildsReserved = 64) noexcept : idxParent(-1){ idxChilds.reserve(numChildsReserved); cost = 0.0; flag = false; pose.fill(0.0); }

        /**
         * @brief Destroy the tree node.
         */
        virtual ~FixedSizeTreeSE2Node() noexcept {}
};


/**
 * @brief This class represents the data structure of a tree with a fixed number of SE(2) nodes.
 * @tparam Tnode The data type of the node to be stored in the tree. This class must be derived from FixedSizeTreeSE2Node.
 */
template <class Tnode> class FixedSizeTreeSE2 {
    static_assert(std::is_base_of<FixedSizeTreeSE2Node,Tnode>::value, "Template class T must be derived from FixedSizeTreeSE2Node!");

    public:
        /**
         * @brief Initialize the tree by initializing the internal containers (allocate memory).
         * @param[in] maxNumNodes Maximum number of nodes to be stored in the tree.
         * @return True if success, false otherwise.
         */
        bool Initialize(int16_t maxNumNodes) noexcept {
            if(maxNumNodes < 2){ // there must be at least one sample and two nodes (root and a sample to be added/removed)
                return false;
            }
            InitializeContainers(maxNumNodes);
            return InitializeRandomNumbers(maxNumNodes);
        }

        /**
         * @brief Terminate the tree by clearing the internal containers (free memory).
         */
        void Terminate(void) noexcept {
            nodes.resize(0);
            activeIndices.resize(0);
            freeIndices.resize(0);
            randomNumber.resize(0);
            lutR3.resize(0);
            lutOptimalRadius.resize(0);
            idxRandomNumber = 0;
            idxRoot = 0;
        }

        /**
         * @brief Set the C space volume and calculate the optimal radius (distance metric) for nearest neighbors search.
         * @param[in] volumeCSpace The volume of the C space according to (x) * (y) * (wpsi*psi), where wpsi denotes the weighting of psi in the metric distance function.
         * @note IMPORTANT: MAKE SURE THAT YOU INITIALIZED THE TREE BY CALLING THE @ref INITIALIZE MEMBER FUNCTION TO AVOID ACCESS VIOLATIONS!
         */
        void SetCSpaceVolume(double volumeCSpace) noexcept {
            double cbrtV = std::cbrt(volumeCSpace);
            for(size_t n = 0; n < lutOptimalRadius.size(); ++n){
                lutOptimalRadius[n] = cbrtV * lutR3[n];
            }
        }

        /**
         * @brief Set the C space volume based on the sampling area and calculate the optimal radius (distance metric) for nearest neighbors search.
         * @param[in] samplingArea An oriented box representing the area that nodes of the tree will lie in. Make sure that the box area is not zero.
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function.
         * @note IMPORTANT: MAKE SURE THAT YOU INITIALIZED THE TREE BY CALLING THE @ref INITIALIZE MEMBER FUNCTION TO AVOID ACCESS VIOLATIONS!
         */
        void SetCSpaceVolume(const mpsv::geometry::OrientedBox& samplingArea, double metricWeightPsi) noexcept {
            std::array<double,2> dimension = samplingArea.GetDimension();
            double volumeCSpace = dimension[0] * dimension[1] * (metricWeightPsi * 6.28318530717959);
            SetCSpaceVolume(volumeCSpace);
        }

        /**
         * @brief Clear the tree and set an initial pose for the root node.
         * @param[in] rootNode Initial value for the root node.
         * @return The index to the node that has been set as root.
         * @details All nodes are removed and only the root node is added. Node attributes idxParent, idxChilds, cost and flag are reset to default values.
         * @note IMPORTANT: MAKE SURE THAT YOU INITIALIZED THE TREE BY CALLING THE @ref INITIALIZE MEMBER FUNCTION TO AVOID ACCESS VIOLATIONS! THIS MEMBER FUNCTION ASSUMES THAT @ref freeIndices CONTAINS AT LEAST ONE VALUE!
         */
        int16_t ClearAndSetRoot(const Tnode rootNode) noexcept {
            freeIndices.insert(freeIndices.end(), activeIndices.begin(), activeIndices.end());
            activeIndices.clear();
            activeIndices.push_back(freeIndices.back());
            freeIndices.pop_back();
            nodes[activeIndices.back()] = rootNode;
            nodes[activeIndices.back()].idxParent = -1;
            nodes[activeIndices.back()].idxChilds.clear();
            nodes[activeIndices.back()].cost = 0.0;
            nodes[activeIndices.back()].flag = false;
            return (idxRoot = activeIndices.back());
        }

        /**
         * @brief Change the root of the tree by adding a new value for the root node and connecting the root to a new part of the current tree.
         * All branches before the given child node are removed.
         * @param[in] newRootNode The new value for the root node.
         * @param[in] idxChild The child of the root node. The tree remains unchanged from this child node on. All old branches before this child are removed.
         * @param[in] epsPosition Numeric limit (eps) for position to check for equal pose.
         * @param[in] epsAngle Numeric limit (eps) for angle to check for equal pose.
         * @return Index to the new root node or a negative value in case of errors. In the error case the tree remains invalid and must be reset (@ref ClearAndSetRoot).
         * @details YOU MUST MAKE SURE THAT ALL INDICES ARE VALID AND THAT THE TREE CONTAINS AT LEAST ONE NODE TO PREVENT ACCESS VIOLATION!
         * THE COST IS NOT UPDATED! YOU MUST UPDATE OR RECALCULATE THE COST OF THE WHOLE TREE.
         * @note A random leaf node is removed if there's no size in the node container. The removed node might be a former solution leaf node.
         */
        int16_t ChangeRoot(const Tnode newRootNode, int16_t idxChild, const double epsPosition, const double epsAngle) noexcept {
            // Set all flags to true and then set all flags of the branch starting from the child node to false
            for(auto&& a : activeIndices){
                nodes[a].flag = true;
            }
            nodes[idxChild].flag = false;
            SetBranchFlags(idxChild, false);

            // Remove all nodes for which the flag is set
            RemoveMarkedNodes();

            // If new root is too close to the node to be used as child, use it as parent (root) instead of adding a new parent
            std::array<double,3> deltaPose = mpsv::math::PoseDifference(nodes[idxChild].pose, newRootNode.pose);
            if((std::fabs(deltaPose[0]) <= epsPosition) && (std::fabs(deltaPose[1]) <= epsPosition) && (std::fabs(deltaPose[2]) <= epsAngle)){
                nodes[idxChild].idxParent = -1;
                return idxChild;
            }

            // If the tree is full, remove a random leaf
            if(IsFull()){
                if(!RemoveRandomLeaf(-1)){
                    return -1;
                }
            }

            // Add new root
            int16_t idxNewRoot = freeIndices.back();
            freeIndices.pop_back();
            activeIndices.push_back(idxNewRoot);
            nodes[idxNewRoot] = newRootNode;
            nodes[idxNewRoot].idxParent = -1;
            nodes[idxNewRoot].idxChilds.clear();
            nodes[idxNewRoot].cost = 0.0;
            nodes[idxNewRoot].flag = false;

            // Make connection from new root to child
            nodes[idxChild].idxParent = idxNewRoot;
            nodes[idxNewRoot].idxChilds.push_back(idxChild);
            return (idxRoot = idxNewRoot);
        }

        /**
         * @brief Remove a random leaf node from the tree (except the root node and the solution node).
         * @param[in] idxSolutionNode Index of the current solution node. This node must not be removed.
         * @return True if a leaf has been removed, false otherwise. If a leaf node cannot be removed, then this is the (only) solution node.
         * @details All nodes of the tree (except the root node and the solution node) that have no childs are collected and a random node of this collection is removed.
         */
        bool RemoveRandomLeaf(int16_t idxSolutionNode) noexcept {
            std::vector<int16_t> idxActiveLeafs;
            for(size_t a = 0; a < activeIndices.size(); ++a){
                if(nodes[activeIndices[a]].idxChilds.empty() && (nodes[activeIndices[a]].idxParent >= 0) && (activeIndices[a] != idxSolutionNode)){
                    idxActiveLeafs.push_back(a);
                }
            }
            size_t numLeafs = idxActiveLeafs.size();
            if(numLeafs){
                // Select a random index
                size_t a = idxActiveLeafs[std::min(static_cast<size_t>(randomNumber[idxRandomNumber] * static_cast<double>(numLeafs)), numLeafs - 1)];
                idxRandomNumber = (idxRandomNumber + 1) % randomNumber.size();

                // Remove node from childlist of parent
                size_t p = nodes[activeIndices[a]].idxParent;
                for(size_t k = 0; k < nodes[p].idxChilds.size(); ++k){
                    if(nodes[p].idxChilds[k] == activeIndices[a]){
                        nodes[p].idxChilds.erase(nodes[p].idxChilds.begin() + k);
                        break;
                    }
                }

                // Remove activeIndices[a]
                freeIndices.push_back(activeIndices[a]);
                activeIndices.erase(activeIndices.begin() + a);
                return true;
            }
            return false;
        }

        /**
         * @brief Remove all nodes from the tree that have a positive flag.
         * @note IMPORTANT: Those marked nodes are directly removed without checking their branches. Make sure that you marked all branches from the node to be removed to all leafs of this node!
         */
        void RemoveMarkedNodes(void) noexcept {
            // Get indices of all nodes to be removed
            std::vector<int16_t> nodesToRemove;
            for(auto&& a : activeIndices){
                if(nodes[a].flag){
                    nodesToRemove.push_back(a);
                }
            }

            // Remove all values of nodesToRemove from activeIndices and insert them to freeIndices
            if(nodesToRemove.size()){
                std::erase_if(activeIndices, [this](int16_t x){ return this->nodes[x].flag; });
                freeIndices.insert(freeIndices.end(), std::make_move_iterator(nodesToRemove.begin()), std::make_move_iterator(nodesToRemove.end()));
            }
        }

        /**
         * @brief Set the flags of all active nodes.
         * @param[in] flag The flag to be set for all active nodes.
         */
        void SetFlags(bool flag) noexcept {
            for(auto&& i : activeIndices){
                nodes[i].flag = flag;
            }
        }

        /**
         * @brief Set the flags of all child nodes of a branch.
         * @param[in] index The index that indicates the root of the branch. The flag of this value is NOT set!
         * @param[in] flag The flag to be set for all childs and subchilds along the branch starting from "index".
         * @details This member function is called recursively until the flags of all childs and "sub-childs" of the given node have been set.
         * @details IMPORTANT: MAKE SURE THAT @ref index POINTS TO A VALID NODE! THIS VALUE DIRECTLY ACCESSES THE INTERNAL NODE CONTAINER!
         */
        void SetBranchFlags(const int16_t index, bool flag) noexcept {
            for(auto&& i : nodes[index].idxChilds){
                nodes[i].flag = flag;
                SetBranchFlags(i, flag);
            }
        }

        /**
         * @brief Check whether the tree is full (no more nodes can be added) or not.
         * @return True if tree is full, false otherwise.
         */
        bool IsFull(void) noexcept { return freeIndices.empty(); }

        /**
         * @brief Add a new node to the tree.
         * @param[in] idxParent The index of the parent node. Make sure that this index is valid and points to an existing node.
         * @param[in] node The node to be added. The index to the parent is set by this member function and the child container of the node is cleared.
         * @return The index to the added element.
         * @details IMPORTANT: MAKE SURE THAT THE TREE IS NOT FULL! DO NOT CALL THIS FUNCTION IF @ref IsFull RETURNS TRUE!
         */
        int16_t Add(int16_t idxParent, const Tnode& node) noexcept {
            activeIndices.push_back(freeIndices.back());
            freeIndices.pop_back();
            nodes[activeIndices.back()] = node;
            nodes[activeIndices.back()].idxParent = idxParent;
            nodes[activeIndices.back()].idxChilds.clear();
            nodes[idxParent].idxChilds.push_back(activeIndices.back());
            return activeIndices.back();
        }

        /**
         * @brief Find the nearest node.
         * @param[in] node The node value to which to find the nearest node.
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function.
         * @return The index of the nearest node to the input node.
         * @details IMPORTANT: MAKE SURE THAT THE TREE CONTAINS AT LEAST ONE NODE. OTHERWISE THIS FUNCTION RETURNS ALWAYS 0.
         */
        int16_t FindNearestNode(const Tnode& node, const double metricWeightPsi) noexcept {
            int16_t result = 0;
            double distance = std::numeric_limits<double>::infinity();
            double d;
            for(auto&& i : activeIndices){
                d = mpsv::math::DistanceMetricSE2(nodes[i].pose, node.pose, metricWeightPsi);
                if(d < distance){
                    distance = d;
                    result = i;
                }
            }
            return result;
        }

        /**
         * @brief Find the nearest neighbors in a box around a given node.
         * @param[out] idxNeighbors A container of index-values to all nearest neighbors in a sphere with given radius around the given node.
         * @param[in] node The node for which to find the nearest neighbors.
         * @param[in] radius The radius of the sphere according to the distance metric sqrt(x^2 + y^2 + (weightPsi*psi)^2).
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function.
         * @param[in] epsDistanceMetric Numeric threshold to be used to check if two node values are equal.
         * @return True if the given node value already exists in the tree (distance metric is less than or equal to epsDistanceMetric), false otherwise.
         */
        bool FindNearestNeighbors(std::vector<int16_t>& idxNeighbors, const Tnode& node, const double radius, const double metricWeightPsi, const double epsDistanceMetric) noexcept {
            idxNeighbors.clear();
            double R;
            bool result = false;
            for(auto&& i : activeIndices){
                R = mpsv::math::DistanceMetricSE2(nodes[i].pose, node.pose, metricWeightPsi);
                if(R <= radius){
                    idxNeighbors.push_back(i);
                }
                result |= (R <= epsDistanceMetric);
            }
            return result;
        }

        /**
         * @brief Rewire the tree and update costs of the reconnected part.
         * @param[in] idxNodeToRewire Index of the node that should be rewired (index to parent is changed).
         * @param[in] idxNewParent Index to the new parent node.
         * @param[in] additionalCost Additional cost that should be added to the reconnected part.
         * @details IMPORTANT: YOU MUST MAKE SURE THAT THE INDEX VALUE IS VALID TO AVOID ACCESS VIOLATION! THIS MEMBER FUNCTION WILL DIRECTLY ACCESS THE INTERNAL CONTAINER!
         */
        void Rewire(int16_t idxNodeToRewire, int16_t idxNewParent, double additionalCost) noexcept {
            // Remove the node to be rewired from the childs of its old parent
            int16_t indexToOldParent = nodes[idxNodeToRewire].idxParent;
            if(indexToOldParent >= 0){
                for(size_t k = 0; k < nodes[indexToOldParent].idxChilds.size(); ++k){
                    if(idxNodeToRewire == nodes[indexToOldParent].idxChilds[k]){
                        nodes[indexToOldParent].idxChilds.erase(nodes[indexToOldParent].idxChilds.begin() + k);
                        break;
                    }
                }
            }

            // Make the new parent connection and update costs along all childs
            nodes[idxNodeToRewire].idxParent = idxNewParent;
            nodes[idxNewParent].idxChilds.push_back(idxNodeToRewire);
            RecursiveCostUpdate(idxNodeToRewire, additionalCost);
        }

        /**
         * @brief Transform the tree to a new origin (translation only). The pose of all nodes is transformed.
         * @param[in] posOldToNew Position from old origin to new origin.
         */
        void TransformToNewOrigin(std::array<double,2> posOldToNew) noexcept {
            for(auto&& n : nodes){
                n.pose[0] -= posOldToNew[0];
                n.pose[1] -= posOldToNew[1];
            }
        }

        /**
         * @brief Reset the random number counter that is used internally to remove random nodes.
         */
        void ResetRandomNumberCounter(void) noexcept {
            idxRandomNumber = 0;
        }

        /**
         * @brief Get the optimal radius for near neighbor searches or for the step size of local planning based on the current cardinality (number of nodes) of the tree.
         * @return Optimal radius according to the weighted euclidean distance metric sqrt(x^2 + y^2 + (weightPsi * psi)^2).
         * @note IMPORTANT: The optimal radius is stored in an internal look-up table that must be built calling the @ref SetSamplingArea member function!
         */
        double GetOptimalRadius(void) noexcept { return lutOptimalRadius[activeIndices.size()]; }

        /**
         * @brief Get indices to all nodes in the tree.
         * @return A constant reference to the internal container of active nodes.
         */
        const std::vector<int16_t>& GetAllNodeIndices(void) noexcept { return std::cref(activeIndices); }

        /**
         * @brief Get the reference to the internal container of active node indices.
         * @return A reference to the internal container of active nodes.
         */
        std::vector<int16_t>& GetRefNodeIndices(void) noexcept { return std::ref(activeIndices); }

        /**
         * @brief Get the branch to the root.
         * @param[in] index The index from which to start the branch selection.
         * @return The indices from the input index to the root. The first value is the "index" input and the last value is the index of the root node.
         * @details MAKE SURE THAT THE INDEX IS VALID TO PREVENT ACCESS VIOLATION!
         */
        std::vector<int16_t> GetBranchToRoot(int16_t index) noexcept {
            std::vector<int16_t> idxBranch;
            while(index >= 0){
                idxBranch.push_back(index);
                index = nodes[index].idxParent;
            }
            return idxBranch;
        }

        /**
         * @brief Get the path from the root node to a specific node.
         * @param[out] path Path (list of poses) from the root node to that node specified by the index.
         * @param[in] index Index of the node for which to get the path from the root to that node.
         */
        void GetPathFromRootToNode(std::vector<std::array<double,3>>& path, int16_t index) noexcept {
            path.clear();
            while(index >= 0){
                path.push_back(nodes[index].pose);
                index = nodes[index].idxParent;
            }
            std::reverse(path.begin(), path.end());
        }

        /**
         * @brief Get all leaf nodes of a given node.
         * @param[in] index Index of the node for which to get all leaf node indices.
         * @return The indices of all leaf nodes related to the given node.
         * @details MAKE SURE THAT THE INDEX IS VALID TO PREVENT ACCESS VIOLATION!
         */
        std::vector<int16_t> GetAllLeafNodes(int16_t index) noexcept {
            std::vector<int16_t> idxLeafs;
            if(nodes[index].idxChilds.empty()){
                idxLeafs.push_back(index);
            }
            else{
                for(auto&& idxChild : nodes[index].idxChilds){
                    std::vector<int16_t> leafs = GetAllLeafNodes(idxChild);
                    idxLeafs.insert(idxLeafs.end(), std::make_move_iterator(leafs.begin()), std::make_move_iterator(leafs.end()));
                }
            }
            return idxLeafs;
        }

        /**
         * @brief Get the reference to a node.
         * @param[in] index Index of the node from which to get a reference to.
         * @return Reference to a node.
         * @details IMPORTANT: YOU MUST MAKE SURE THAT THE INDEX VALUE IS VALID TO AVOID ACCESS VIOLATION! THIS MEMBER FUNCTION WILL DIRECTLY ACCESS THE INTERNAL CONTAINER!
         */
        Tnode& GetRefNode(int16_t index) noexcept { return std::ref(nodes[index]); }

        /**
         * @brief Get the parent node.
         * @param[in] index Index of the node from which to get the parent.
         * @return Index to the parent node or -1 if index indicates the root node.
         * @details IMPORTANT: YOU MUST MAKE SURE THAT THE INDEX VALUE IS VALID TO AVOID ACCESS VIOLATION! THIS MEMBER FUNCTION WILL DIRECTLY ACCESS THE INTERNAL CONTAINER!
         */
        int16_t GetParentIndex(int16_t index) noexcept { return nodes[index].idxParent; }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        virtual void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept = 0;

        /**
         * @brief Debug function to check whether the tree is correct or not.
         * @details The connection between nodes in all directions are checked. Furthermore it's checked if all nodes can go to the root node via their parents and if the cost reduces along that path.
         */
        void DebugCheck(void) noexcept {
            if(activeIndices.size() + freeIndices.size() > nodes.size()){
                std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): Tree can contain at most " << nodes.size() << " nodes but there're more active (" << activeIndices.size() << ") or free (" << freeIndices.size() << ") indices!\n";
            }
            int numRoots = 0;
            for(auto&& a : activeIndices){
                // Check how many roots are in the tree
                if(nodes[a].idxParent < 0){
                    numRoots++;
                }

                // Check for negative cost value
                if(nodes[a].cost < 0.0){
                    std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): The cost of node[" << a << "] is negative!\n";
                }

                // Try to reach the root and check for cost reduction along that path
                int16_t idx = a;
                double c = nodes[idx].cost;
                for(size_t n = 0; (n < nodes.size()) && (idx >= 0); ++n){
                    idx = nodes[idx].idxParent;
                    if(idx >= 0){
                        if(nodes[idx].cost >= c){
                            std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): The cost from root to node[" << a << "] does not monotonically increase (" << c << " !> " << nodes[idx].cost << ")!\n";
                        }
                        c = nodes[idx].cost;
                    }
                }
                if(idx >= 0){
                    std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): node[" << a << "] cannot reach the root!\n";
                }

                // Check if this node is registered as child in its parent
                if(nodes[a].idxParent >= 0){
                    idx = nodes[a].idxParent;
                    int val = 0;
                    for(auto&& c : nodes[idx].idxChilds){
                        val += static_cast<int>(c == a);
                    }
                    if(val != 1){
                        std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): node[" << a << "] is registered " << val << " times as child (parent node[" << idx << "])!\n";
                    }
                }

                // Check if all childs of this node have this node as parent
                for(auto&& c : nodes[a].idxChilds){
                    if(a != nodes[c].idxParent){
                        std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): node[" << c << "] should have node[" << a << "] as parent but has node[" << nodes[a].idxParent << "] as parent!\n";
                    }
                }
            }
            if(1 != numRoots){
                std::cerr << "### ERROR in FixedSizeTreeSE2::DebugCheck(): There are " << numRoots << " roots in the tree! There must be only one root!\n";
            }
        }

    protected:
        std::vector<Tnode> nodes;               // Main container of nodes. The size is set during initialization.
        std::vector<int16_t> activeIndices;     // Variable size container of indices to active nodes in @ref nodes container. Some space is reserved during initialization.
        std::vector<int16_t> freeIndices;       // Variable size container of indices to free spaces in @ref nodes container. The size and content is set during initialization.
        std::vector<double> randomNumber;       // A container of precomputed random numbers. Those random numbers are used to remove random leaf nodes from the tree using the @ref RemoveRandomLeaf member function.
        std::vector<double> lutR3;              // Look-up table containing values to be used to calculate the step-size for the near neighbor search. Optimal radius is given by r = Vc^(1/3) * lutR3[n] with n being the cardinality of the tree and Vc being the volume of the search space.
        std::vector<double> lutOptimalRadius;   // Look-up table containing the optimal radius for nearest neighbors searches based on the current cardinality (number of nodes) of the tree. This value is based on the volume of the search space.
        size_t idxRandomNumber;                 // Index to current random number in @ref randomNumber to be used.
        int16_t idxRoot;                        // Index to the root node.

        /**
         * @brief Initialize containers of nodes and indices.
         * @param maxNumNodes Maximum number of nodes to be stored in the tree. Must be greater than one!
         */
        void InitializeContainers(int16_t maxNumNodes) noexcept {
            size_t capacity = static_cast<size_t>(maxNumNodes);
            nodes.resize(capacity, Tnode(capacity - 1));
            activeIndices.reserve(capacity);
            activeIndices.clear();
            freeIndices.resize(capacity);
            lutR3.resize(capacity);
            lutOptimalRadius.resize(capacity);
            for(size_t n = 0; n < capacity; ++n){
                freeIndices[n] = static_cast<int16_t>(capacity - 1 - n);
                double k = static_cast<double>(std::max(static_cast<size_t>(3), n));
                lutR3[n] = std::cbrt(2.54647908947033 * std::log(k) / k); // 2.54647908947033 = 2^d*(1+1/d)/(4/3 * pi), with d=3
                lutOptimalRadius[n] = 0.0; // actual value set by @ref SetSamplingArea whenever the volume of the C space changes
            }
        }

        /**
         * @brief Initialize the container of random numbers to be used for random leaf removal.
         * @param maxNumNodes Maximum number of nodes to be stored in the tree. Must be greater than one!
         * @return True if success, false otherwise.
         */
        bool InitializeRandomNumbers(int16_t maxNumNodes) noexcept {
            uint32_t u = static_cast<uint32_t>(maxNumNodes) * 2 - 3; // use the next power of 2 of 2*maxNumNodes
            u |= u >> 1;
            u |= u >> 2;
            u |= u >> 4;
            u |= u >> 8;
            u |= u >> 16;
            idxRandomNumber = 0;
            size_t N = static_cast<size_t>(++u);
            mpsv::sampler::HaltonSequence hs;
            if(!hs.Initialize(1, N)){
                return false;
            }
            randomNumber.resize(N);
            for(size_t n = 0; n < N; ++n){
                randomNumber[n] = hs.Sample(0, n);
            }
            return true;
        }

        /**
         * @brief Make a cost update for a node and all its child nodes.
         * @param[in] index The index to a node for which the cost update should be done.
         * @param[in] additionalCost An additional cost value that is added to the cost of the indexed node and all its childs.
         * @details IMPORTANT: YOU MUST MAKE SURE THAT THE INDEX VALUE IS VALID TO AVOID ACCESS VIOLATION! THIS MEMBER FUNCTION WILL DIRECTLY ACCESS THE INTERNAL CONTAINER!
         */
        void RecursiveCostUpdate(int16_t index, double additionalCost) noexcept {
            nodes[index].cost += additionalCost;
            for(auto&& i : nodes[index].idxChilds){
                RecursiveCostUpdate(i, additionalCost);
            }
        }
};


} /* namespace: core */


} /* namespace: mpsv */

