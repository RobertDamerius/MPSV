#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/math/Metric.hpp>
#include <mpsv/sampler/HaltonSequence.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/diagnostics/DataLogFile.hpp>


namespace mpsv {


namespace core {


/**
 * @brief This class represents the base class for a node data type to be used by the FixedSizeTreeSE2.
 */
class FixedSizeTreeSE2Node {
    public:
        std::array<double, 3> pose{0.0, 0.0, 0.0};   // Pose in SE2 space: {x, y, psi}.
        double cost{0.0};                            // Cumulative cost.
        int16_t iParent{-1};                         // Index of parent node.
        int16_t iFirstChild{-1};                     // Index of first child node.
        int16_t iPrevSibling{-1};                    // Index of previous sibling node.
        int16_t iNextSibling{-1};                    // Index of next sibling node.
        int16_t iLeafList{-1};                       // Index in leaf list.
        int16_t iActiveList{-1};                     // Index in active list.

        /**
         * @brief Create a tree node with default values.
         */
        FixedSizeTreeSE2Node() noexcept = default;

        /**
         * @brief Create a tree node with the given pose.
         * @tparam T The type of the pose. Must be std::array<double,N> with N >= 3!
         * @param[in] pose The pose in SE2 space: {x, y, psi}.
         * @param[in] cost The cumulative cost. Defaults to 0.0.
         */
        template <class T> explicit FixedSizeTreeSE2Node(const T& pose, double cost = 0.0) noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {pose} must be of type std::array<double,N> with N >= 3!");
            std::ranges::copy(pose | std::views::take(3), this->pose.begin());
            this->cost = cost;
        }
};


/**
 * @brief This class represents the data structure of a tree with a fixed number of SE(2) nodes.
 * @tparam Tnode The data type of the node to be stored in the tree. This class must be derived from FixedSizeTreeSE2Node.
 */
template <class Tnode> class FixedSizeTreeSE2 {
    static_assert(std::is_base_of<FixedSizeTreeSE2Node,Tnode>::value, "Template class T must be derived from FixedSizeTreeSE2Node!");

    public:
        /**
         * @brief Construct a new tree with a fixed number of SE(2) nodes.
         */
        FixedSizeTreeSE2(){ Clear(); SetNumericLimit(0.0); }

        /**
         * @brief Clear the tree to an empty graph and resize the internal storage (at least one node).
         * @param[in] maxNumNodes Maximum number of nodes to be stored in the tree.
         */
        void Clear(int16_t maxNumNodes = 1) noexcept {
            maxNumNodes = std::max<int16_t>(1, maxNumNodes);
            freeIndices.clear();
            freeIndices.reserve(maxNumNodes);
            for(int16_t i = maxNumNodes - 1; i >= 0; --i){
                freeIndices.push_back(i);
            }
            activeIndices.clear();
            activeIndices.reserve(maxNumNodes);
            leafIndices.clear();
            leafIndices.reserve(maxNumNodes);
            traversalStack.clear();
            traversalStack.reserve(maxNumNodes);
            nodes.resize(maxNumNodes);
            rootNodeIndex = -1;
            lutR3.resize(maxNumNodes);
            lutOptimalRadius.resize(maxNumNodes);
            lutRandom.resize(maxNumNodes);
            std::mt19937_64 gen(0);
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            for(int16_t n = 0; n < maxNumNodes; ++n){
                double k = static_cast<double>(std::max<int16_t>(3, n));
                lutR3[n] = std::cbrt(2.54647908947033 * std::log(k) / k); // 2.54647908947033 = 2^d*(1+1/d)/(4/3 * pi), with d=3
                lutOptimalRadius[n] = std::numeric_limits<double>::infinity(); // actual value set by @ref SetCSpaceVolume
                lutRandom[n] = dist(gen);
            }
            idxRandom = 0;
        }

        /**
         * @brief Reset the tree with a given initial pose.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 3.
         * @param[in] initialPose The initial pose for the tree.
         * @return The index of the added root node, or -1 if the operation failed.
         */
        template <class T> int16_t Reset(const T& initialPose) noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {initialPose} must be of type std::array<double,N> with N >= 3!");
            int16_t numNodes = static_cast<int16_t>(nodes.size());
            freeIndices.resize(numNodes);
            for(int16_t i = 0; i < numNodes; ++i){
                freeIndices[i] = numNodes - 1 - i;
            }
            activeIndices.clear();
            leafIndices.clear();
            Tnode node(initialPose);
            rootNodeIndex = Add(-1, node);
            return rootNodeIndex;
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
         * @return The index to the added node, or -1 if the operation failed.
         */
        int16_t Add(int16_t idxParent, const Tnode& node) noexcept {
            if(freeIndices.empty()) return -1;
            int16_t idx = freeIndices.back();
            freeIndices.pop_back();
            nodes[idx].pose = node.pose;
            nodes[idx].cost = node.cost;
            nodes[idx].iParent = idxParent;
            nodes[idx].iFirstChild = -1;
            nodes[idx].iPrevSibling = -1;
            nodes[idx].iNextSibling = -1;
            AddToActiveList(idx);
            AddToLeafList(idx);
            if(idxParent >= 0){
                if(nodes[idxParent].iFirstChild < 0){
                    RemoveFromLeafList(idxParent);
                }
                int16_t oldFirstChild = nodes[idxParent].iFirstChild;
                nodes[idx].iNextSibling = oldFirstChild;
                if(oldFirstChild >= 0){
                    nodes[oldFirstChild].iPrevSibling = idx;
                }
                nodes[idxParent].iFirstChild = idx;
            }
            return idx;
        }

        /**
         * @brief Find the nearest node.
         * @param[in] node The node value to which to find the nearest node.
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function.
         * @return The index of the nearest node to the input node or -1 if no suitable node is found.
         */
        int16_t FindNearestNode(const Tnode& node, const double metricWeightPsi) noexcept {
            int16_t result = -1;
            double distance = std::numeric_limits<double>::infinity();
            for(auto&& i : activeIndices){
                double d = mpsv::math::DistanceMetricSE2(nodes[i].pose, node.pose, metricWeightPsi);
                if(d < distance){
                    distance = d;
                    result = i;
                }
            }
            return result;
        }

        /**
         * @brief Find the nearest neighbors around a given node.
         * @param[out] idxNeighbors A container of index-values to all nearest neighbors in a sphere with given radius around the given node.
         * @param[in] node The node for which to find the nearest neighbors.
         * @param[in] radius The radius of the sphere according to the distance metric sqrt(x^2 + y^2 + (weightPsi*psi)^2). Use @ref GetOptimalRadius to get the optimal radius.
         * @param[in] metricWeightPsi Weighting for psi in the metric distance function.
         * @return True if the given node value already exists in the tree (distance metric is less than or equal to the numeric limit set via @ref SetNumericLimit), false otherwise.
         */
        bool FindNearestNeighbors(std::vector<int16_t>& idxNeighbors, const Tnode& node, const double radius, const double metricWeightPsi) noexcept {
            idxNeighbors.clear();
            bool result = false;
            for(auto&& i : activeIndices){
                double R = mpsv::math::DistanceMetricSE2(nodes[i].pose, node.pose, metricWeightPsi);
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
         */
        void Rewire(int16_t idxNodeToRewire, int16_t idxNewParent, double additionalCost) noexcept {
            // Remove the node to be rewired from the childs of its old parent
            DetachFromParent(idxNodeToRewire);

            // Make the new parent connection
            if(nodes[idxNewParent].iFirstChild < 0){
                RemoveFromLeafList(idxNewParent);
            }
            nodes[idxNodeToRewire].iParent = idxNewParent;
            nodes[idxNodeToRewire].iPrevSibling = -1;
            nodes[idxNodeToRewire].iNextSibling = nodes[idxNewParent].iFirstChild;
            if(nodes[idxNewParent].iFirstChild >= 0){
                nodes[nodes[idxNewParent].iFirstChild].iPrevSibling = idxNodeToRewire;
            }
            nodes[idxNewParent].iFirstChild = idxNodeToRewire;

            // Update cost for all child branches
            traversalStack.clear();
            traversalStack.push_back(idxNodeToRewire);
            while(!traversalStack.empty()){
                int16_t current = traversalStack.back();
                traversalStack.pop_back();
                nodes[current].cost += additionalCost;
                int16_t child = nodes[current].iFirstChild;
                while(child >= 0){
                    traversalStack.push_back(child);
                    child = nodes[child].iNextSibling;
                }
            }
        }

        /**
         * @brief Remove a random leaf node from the tree (except the root node and the solution node).
         * @param[in] idxSolutionNode Index of the current solution node. This node must not be removed.
         * @return True if a leaf has been removed, false otherwise. If a leaf node cannot be removed, then this is the (only) solution node.
         */
        bool RemoveRandomLeaf(int16_t idxSolutionNode) noexcept {
            // Select a random leaf index to be removed
            size_t numLeaves = leafIndices.size();
            size_t removeLeafIndex = static_cast<size_t>(lutRandom[idxRandom] * numLeaves);
            idxRandom = (idxRandom + 1) % lutRandom.size();

            // Find a leaf that is not the solution node or the root node
            for(size_t i = 0; i < numLeaves; ++i){
                if((leafIndices[removeLeafIndex] != idxSolutionNode) && (leafIndices[removeLeafIndex] != rootNodeIndex)){
                    int16_t idxRemove = leafIndices[removeLeafIndex];
                    DetachFromParent(idxRemove);
                    RemoveFromLeafList(idxRemove);
                    RemoveFromActiveList(idxRemove);
                    freeIndices.push_back(idxRemove);
                    return true;
                }

                // Try the next leaf
                removeLeafIndex = (removeLeafIndex + 1) % numLeaves;
            }

            // Error: no leaf could be removed
            return false;
        }

        /**
         * @brief Delete the subtree for a given node index that is not the root node of the tree.
         * @param[in] nodeIndex The index to the root of the subtree to be removed.
         */
        void DeleteSubtree(int16_t nodeIndex) noexcept {
            if((nodeIndex < 0) || (nodeIndex >= static_cast<int16_t>(nodes.size())) || (nodeIndex == rootNodeIndex) || (nodes[nodeIndex].iActiveList < 0)){
                return;
            }
            DetachFromParent(nodeIndex);
            traversalStack.clear();
            traversalStack.push_back(nodeIndex);
            while(!traversalStack.empty()){
                int16_t current = traversalStack.back();
                traversalStack.pop_back();
                int16_t child = nodes[current].iFirstChild;
                while(child >= 0){
                    traversalStack.push_back(child);
                    child = nodes[child].iNextSibling;
                }
                if(nodes[current].iLeafList >= 0){
                    RemoveFromLeafList(current);
                }
                RemoveFromActiveList(current);
                freeIndices.push_back(current);
            }
        }

        /**
         * @brief Change the root of the tree by adding a new value for the root node and connecting the root to a new part of the current tree.
         * All branches before the given node are removed.
         * @tparam T The type of the pose. Must be std::array<double,N> with N >= 3!
         * @param[in] newRootPose The pose of the new root node.
         * @param[in] childIndex The index of the child to be connected to the root node. The tree remains unchanged from this child node on. All old branches before this child are removed.
         * If this value is invalid, @ref Reset is called.
         * @return Index to the new root node.
         * @note A random leaf node is removed if there's no size in the node container, except for the node specified by childIndex. If no random leaf could be removed, @ref Reset is called.
         */
        template <class T> int16_t ChangeRoot(const T& newRootPose, int16_t childIndex, const double metricWeightPsi) noexcept {
            static_assert(mpsv::is_vec3<T>::value,"Argument {newRootPose} must be of type std::array<double,N> with N >= 3!");
            if((childIndex < 0) || (childIndex >= static_cast<int16_t>(nodes.size())) || (nodes[childIndex].iActiveList < 0)){
                return Reset(newRootPose);
            }
            const int16_t oldRootIndex = rootNodeIndex;
            int16_t newRootIndex = -1;

            // Decide whether to use an existing node (close enough) or generate a new one
            if(mpsv::math::DistanceMetricSE2(newRootPose, nodes[childIndex].pose, metricWeightPsi) <= epsDistanceMetric){
                // Reuse existing node as new root
                newRootIndex = childIndex;
                std::ranges::copy(newRootPose | std::views::take(3), nodes[newRootIndex].pose.begin());
                nodes[newRootIndex].cost = 0.0;

                // Detach from its old parent
                DetachFromParent(newRootIndex);
            } else {
                if(IsFull()){
                    if(!RemoveRandomLeaf(childIndex)){
                        return Reset(newRootPose);
                    }
                }
                newRootIndex = freeIndices.back();
                freeIndices.pop_back();
                AddToActiveList(newRootIndex);

                // Initialize new root
                std::ranges::copy(newRootPose | std::views::take(3), nodes[newRootIndex].pose.begin());
                nodes[newRootIndex].cost = 0.0;
                nodes[newRootIndex].iParent = -1;
                nodes[newRootIndex].iFirstChild = -1;
                nodes[newRootIndex].iPrevSibling = -1;
                nodes[newRootIndex].iNextSibling = -1;
                AddToLeafList(newRootIndex);

                // Rewire the target node to the new root with 0.0 additional cost
                Rewire(childIndex, newRootIndex, 0.0);
            }

            // Prune everything that remained under the old root before childIndex was disconnected.
            rootNodeIndex = newRootIndex;
            if(oldRootIndex != childIndex){
                DeleteSubtree(oldRootIndex);
            }
            return rootNodeIndex;
        }

        /**
         * @brief Set the C space volume and calculate the optimal radius (distance metric) for nearest neighbors search.
         * @param[in] volumeCSpace The volume of the C space according to (x) * (y) * (wpsi*psi), where wpsi denotes the weighting of psi in the metric distance function.
         */
        void SetCSpaceVolume(double volumeCSpace) noexcept {
            double cbrtV = std::cbrt(std::fabs(volumeCSpace));
            for(size_t n = 0; n < lutOptimalRadius.size(); ++n){
                lutOptimalRadius[n] = cbrtV * lutR3[n];
            }
        }

        /**
         * @brief Set absolute numeric limit for distance metric. This limit is used to check for equal poses.
         * @param[in] epsDistanceMetric Numeric limit for the distance metric.
         */
        void SetNumericLimit(const double epsDistanceMetric) noexcept {
            this->epsDistanceMetric = std::max(std::numeric_limits<double>::epsilon(), epsDistanceMetric);
        }

        /**
         * @brief Get the optimal radius for near neighbor searches based on the current cardinality (number of nodes) of the tree.
         * @return Optimal radius according to the weighted euclidean distance metric sqrt(x^2 + y^2 + (weightPsi * psi)^2).
         * @note The volume of the configuration space has to be set in advance via @ref SetCSpaceVolume.
         */
        double GetOptimalRadius(void) noexcept { return lutOptimalRadius[std::min(activeIndices.size(), lutOptimalRadius.size() - 1)]; }

        /**
         * @brief Transform the tree to a new origin (translation only). The poses of all active nodes are transformed.
         * @param[in] posOldToNew Position from old origin to new origin.
         */
        void TransformToNewOrigin(std::array<double,2> posOldToNew) noexcept {
            for(auto&& i : activeIndices){
                nodes[i].pose[0] -= posOldToNew[0];
                nodes[i].pose[1] -= posOldToNew[1];
            }
        }

        /* Non-const operator[]: allows modifying the node */
        Tnode& operator[](int16_t index) noexcept { return nodes[index]; }

        /* Const operator[]: allows reading the node when the tree is const */
        const Tnode& operator[](int16_t index) const noexcept { return nodes[index]; }

        /**
         * @brief Get indices to all active nodes in the tree.
         * @return A constant reference to the internal container of active nodes.
         */
        const std::vector<int16_t>& GetActiveIndices(void) const noexcept { return activeIndices; }

        /**
         * @brief Get indices to all leaf nodes in the tree.
         * @return A constant reference to the internal container of leaf nodes.
         */
        const std::vector<int16_t>& GetLeafIndices(void) const noexcept { return leafIndices; }

        /**
         * @brief Get all leaf indices of a specific node in the tree.
         * @param[out] leafIndicesOut A vector to store the leaf indices.
         * @param nodeIndex The index of the node for which to get leaf indices.
         */
        void GetLeafIndices(std::vector<int16_t>& leafIndicesOut, int16_t nodeIndex) noexcept{
            leafIndicesOut.clear();
            traversalStack.clear();
            if((nodeIndex < 0) || (nodeIndex >= static_cast<int16_t>(nodes.size())) || (nodes[nodeIndex].iActiveList < 0)){
                return;
            }
            traversalStack.push_back(nodeIndex);
            while(!traversalStack.empty()){
                int16_t current = traversalStack.back();
                traversalStack.pop_back();
                if(nodes[current].iFirstChild < 0){
                    leafIndicesOut.push_back(current);
                    continue;
                }
                int16_t child = nodes[current].iFirstChild;
                while(child >= 0){
                    traversalStack.push_back(child);
                    child = nodes[child].iNextSibling;
                }
            }
        }

        /**
         * @brief Get the path from the root node to a specific node.
         * @param[out] path Path (list of poses) from the root node to that node specified by the index.
         * @param[in] nodeIndex Index of the node for which to get the path from the root to that node.
         */
        void GetPathFromRootToNode(std::vector<std::array<double, 3>>& path, int16_t nodeIndex) noexcept {
            path.clear();
            if(nodeIndex < 0 || nodeIndex >= static_cast<int16_t>(nodes.size()) || (nodes[nodeIndex].iActiveList < 0)){
                return;
            }
            traversalStack.clear();
            while(nodeIndex >= 0){
                traversalStack.push_back(nodeIndex);
                nodeIndex = nodes[nodeIndex].iParent;
            }
            path.reserve(traversalStack.size());
            for(auto it = traversalStack.rbegin(); it != traversalStack.rend(); ++it){
                path.push_back(nodes[*it].pose);
            }
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        virtual void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            std::vector<double> vecPoses;
            std::vector<double> vecCost;
            std::vector<int16_t> vecParentIndices;
            uint32_t Na = static_cast<uint32_t>(activeIndices.size());
            uint32_t Nn = static_cast<uint32_t>(nodes.size());
            vecPoses.reserve(3 * Nn);
            vecCost.reserve(Nn);
            vecParentIndices.reserve(Nn);
            for(uint32_t n = 0; n != Nn; ++n){
                vecPoses.push_back(nodes[n].pose[0]);
                vecPoses.push_back(nodes[n].pose[1]);
                vecPoses.push_back(nodes[n].pose[2]);
                vecCost.push_back(nodes[n].cost);
                vecParentIndices.push_back(nodes[n].iParent);
            }
            if(Nn){
                file.WriteField("double", preString + "pose", {3, Nn}, &vecPoses[0], sizeof(double) * vecPoses.size());
                file.WriteField("int16_t", preString + "iParent", {Nn}, &vecParentIndices[0], sizeof(int16_t) * vecParentIndices.size());
                file.WriteField("double", preString + "cost", {Nn}, &vecCost[0], sizeof(double) * vecCost.size());
                file.WriteField("int16_t", preString + "activeIndices", {Na}, &activeIndices[0], sizeof(int16_t) * activeIndices.size());
            }
        }

    protected:
        std::vector<int16_t> freeIndices;                                   // Stores the indices of free nodes in the tree.
        std::vector<int16_t> activeIndices;                                 // Stores the indices of active nodes in the tree.
        std::vector<int16_t> leafIndices;                                   // Stores the indices of leaf nodes in the tree.
        std::vector<int16_t> traversalStack;                                // Pre-allocated stack for traversing through branches.
        std::vector<Tnode> nodes;                                           // Internal storage for all nodes.
        std::vector<double> lutR3;                                          // Look-up table containing values to be used to calculate the step-size for the near neighbor search. Optimal radius is given by r = Vc^(1/3) * lutR3[n] with n being the cardinality of the tree and Vc being the volume of the search space.
        std::vector<double> lutOptimalRadius;                               // Look-up table containing the optimal radius for nearest neighbors searches based on the current cardinality (number of nodes) of the tree. This value is based on the volume of the search space.
        std::vector<double> lutRandom;                                      // A container of precomputed random numbers. Those random numbers are used to remove random leaf nodes from the tree.
        size_t idxRandom{0};                                                // Current index of the random number.
        int16_t rootNodeIndex{-1};                                          // Index of the root node.
        double epsDistanceMetric{std::numeric_limits<double>::epsilon()};   // Numeric limit for distance metric.

        /**
         * @brief Add a node to the active list.
         * @param[in] idx Index of the node to be added to the active list.
         */
        void AddToActiveList(int16_t idx) noexcept {
            nodes[idx].iActiveList = static_cast<int16_t>(activeIndices.size());
            activeIndices.push_back(idx);
        }

        /**
         * @brief Add a node to the leaf list.
         * @param[in] idx Index of the node to be added to the leaf list.
         */
        void AddToLeafList(int16_t idx) noexcept {
            nodes[idx].iLeafList = static_cast<int16_t>(leafIndices.size());
            leafIndices.push_back(idx);
        }

        /**
         * @brief Remove a node from the active list.
         * @param[in] idx Index of the node to be removed from the active list.
         */
        void RemoveFromActiveList(int16_t idx) noexcept {
            nodes[activeIndices.back()].iActiveList = nodes[idx].iActiveList;
            activeIndices[nodes[activeIndices.back()].iActiveList] = activeIndices.back();
            nodes[idx].iActiveList = -1;
            activeIndices.pop_back();
        }

        /**
         * @brief Remove a node from the leaf list.
         * @param[in] idx Index of the node to be removed from the leaf list.
         */
        void RemoveFromLeafList(int16_t idx) noexcept {
            nodes[leafIndices.back()].iLeafList = nodes[idx].iLeafList;
            leafIndices[nodes[leafIndices.back()].iLeafList] = leafIndices.back();
            nodes[idx].iLeafList = -1;
            leafIndices.pop_back();
        }

        /**
         * @brief Detach a node from its parent, remove it from the parent's child list, and update the parent's leaf status.
         * @param[in] nodeIndex Index of the node to be detached from its parent.
         */
        void DetachFromParent(int16_t nodeIndex) noexcept {
            int16_t parentIndex = nodes[nodeIndex].iParent;
            if(parentIndex < 0) return;
            int16_t prev = nodes[nodeIndex].iPrevSibling;
            int16_t next = nodes[nodeIndex].iNextSibling;
            if(prev < 0){
                nodes[parentIndex].iFirstChild = next;
            } else {
                nodes[prev].iNextSibling = next;
            }
            if(next >= 0){
                nodes[next].iPrevSibling = prev;
            }
            if(nodes[parentIndex].iFirstChild < 0){
                AddToLeafList(parentIndex);
            }
            nodes[nodeIndex].iParent = -1;
            nodes[nodeIndex].iPrevSibling = -1;
            nodes[nodeIndex].iNextSibling = -1;
        }
};


} /* namespace: core */


} /* namespace: mpsv */

