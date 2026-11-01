function log = ReadLogFile(filename)
    %mpsv.ReadLogFile Read and decode a log file.
    % 
    % PARAMETER
    % filename   ... Name of the logfile.
    assert(ischar(filename), 'ERROR: Input "filename" must be a string!');
    file = fopen(filename,'rb');
    assert(file >= 0, 'ERROR: Could not open logfile "%s"!', filename);
    fileCleanup = onCleanup(@() fclose(file));
    status = fseek(file, 0, 'eof');
    assert(status == 0, 'ERROR: Could not seek to the end of the logfile!');
    fileSize = ftell(file);
    assert((fileSize >= 0) && (fileSize <= flintmax), 'ERROR: Could not determine logfile size!');
    status = fseek(file, 0, 'bof');
    assert(status == 0, 'ERROR: Could not seek to the start of the logfile!');
    startPatternCounter = int32(0);
    fieldPatternCounter = int32(0);
    logNumber = 0;
    log = cell.empty();
    while(true)
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % Search for pattern ($$: start of header) (##: start of field)
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        [B,L] = fread(file, 1, 'uint8');
        if(1 ~= L), break; end
        if(uint8('$') == uint8(B))
            startPatternCounter = startPatternCounter + int32(1);
            fieldPatternCounter = int32(0);
        elseif(uint8('#') == uint8(B))
            startPatternCounter = int32(0);
            fieldPatternCounter = fieldPatternCounter + int32(1);
        end

        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % Header data
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(int32(2) == startPatternCounter)
            startPatternCounter = int32(0);
            logNumber = logNumber + 1;
            [V,L] = fread(file,1,'uint32'); assert(L == 1, 'ERROR: Invalid header data format!');
            headerSize = uint32(V);
            if(headerSize >= uint32(4))
                headerSize = headerSize - uint32(4);
                [V,L] = fread(file,1,'uint32'); assert(L == 1, 'ERROR: Invalid header data format!');
                log{logNumber}.info.year = uint32(V);
            end
            if(headerSize >= uint32(1))
                headerSize = headerSize - uint32(1);
                [V,L] = fread(file,1,'uint8'); assert(L == 1, 'ERROR: Invalid header data format!');
                log{logNumber}.info.month = uint8(V);
            end
            if(headerSize >= uint32(1))
                headerSize = headerSize - uint32(1);
                [V,L] = fread(file,1,'uint8'); assert(L == 1, 'ERROR: Invalid header data format!');
                log{logNumber}.info.day = uint8(V);
            end
            if(headerSize >= uint32(8))
                headerSize = headerSize - uint32(8);
                [V,L] = fread(file,1,'double'); assert(L == 1, 'ERROR: Invalid header data format!');
                log{logNumber}.info.timestampUTC = double(V);
            end
            if(headerSize)
                [~,L] = fread(file,headerSize,'uint8'); assert(L == headerSize, 'ERROR: Invalid header data format!');
            end
        end

        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        % Field data
        % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(int32(2) == fieldPatternCounter)
            fieldPatternCounter = int32(0);
            
            % Field size
            [V,L] = fread(file,1,'uint64'); assert(L == 1, 'ERROR: Invalid field data format!');
            fieldSize = uint64(V);
            assert(fieldSize <= uint64(flintmax), 'ERROR: Invalid field data format!');
            fieldDataStart = ftell(file);
            assert((fieldDataStart >= 0) && (double(fieldSize) <= (fileSize - fieldDataStart)), 'ERROR: Invalid field data format!');

            % Type
            assert(fieldSize >= uint64(4), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(4);
            [V,L] = fread(file,1,'uint32'); assert(L == 1, 'ERROR: Invalid field data format!');
            lengthOfType = uint32(V);
            assert(fieldSize >= uint64(lengthOfType), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(lengthOfType);
            [V,L] = fread(file,lengthOfType,'char'); assert(L == lengthOfType, 'ERROR: Invalid field data format!');
            dataType = ToInternalDataType(char(V));

            % Name
            assert(fieldSize >= uint64(4), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(4);
            [V,L] = fread(file,1,'uint32'); assert(L == 1, 'ERROR: Invalid field data format!');
            lengthOfName = uint32(V);
            assert(fieldSize >= uint64(lengthOfName), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(lengthOfName);
            [V,L] = fread(file,lengthOfName,'char'); assert(L == lengthOfName, 'ERROR: Invalid field data format!');
            name = char(V);
            name = reshape(name,[1 numel(name)]);
            
            % Dimension
            assert(fieldSize >= uint64(4), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(4);
            [V,L] = fread(file,1,'uint32'); assert(L == 1, 'ERROR: Invalid field data format!');
            numberOfDimensions = uint32(V);
            dimensionDataSize = uint64(numberOfDimensions) * uint64(4);
            assert((numberOfDimensions > 0) && (numberOfDimensions <= 32) && (fieldSize >= dimensionDataSize), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - dimensionDataSize;
            [V,L] = fread(file,numberOfDimensions,'uint32'); assert(L == numberOfDimensions, 'ERROR: Invalid field data format!');
            dimension = uint32(V)';
            numberOfElements = uint64(1);
            for d = dimension
                assert((d == 0) || (numberOfElements <= idivide(intmax('uint64'), uint64(d), 'floor')), 'ERROR: Invalid field dimensions!');
                numberOfElements = numberOfElements * uint64(d);
            end

            % Value
            assert(fieldSize >= uint64(8), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(8);
            [V,L] = fread(file,1,'uint64'); assert(L == 1, 'ERROR: Invalid field data format!');
            sizeofValue = uint64(V);
            elementSize = BytesPerElement(dataType);
            assert(numberOfElements <= idivide(intmax('uint64'), elementSize, 'floor'), 'ERROR: Invalid field dimensions!');
            expectedValueSize = numberOfElements * elementSize;
            assert((sizeofValue == expectedValueSize) && (fieldSize >= sizeofValue), 'ERROR: Invalid field data format!');
            assert(numberOfElements <= uint64(flintmax), 'ERROR: Field contains too many values to load!');
            fieldSize = fieldSize - sizeofValue;
            if(strcmp('boolean',dataType))
                [V,L] = fread(file,dimension,'uint8'); assert(uint64(L) == numberOfElements, 'ERROR: Invalid field data format!');
                eval(['log{logNumber}.',name,' = ' dataType '(V > 0);']);
            else
                [V,L] = fread(file,dimension,dataType); assert(uint64(L) == numberOfElements, 'ERROR: Invalid field data format!');
                eval(['log{logNumber}.',name,' = ' dataType '(V);']);
            end

            % Unknown field data
            if(fieldSize)
                [~,L] = fread(file,double(fieldSize),'uint8');
                assert(uint64(L) == fieldSize, 'ERROR: Invalid field data format!');
            end
        end
    end
end

function elementSize = BytesPerElement(dataType)
    switch(dataType)
        case {'boolean','char','int8','uint8'}
            elementSize = uint64(1);
        case {'int16','uint16'}
            elementSize = uint64(2);
        case {'int32','uint32','single'}
            elementSize = uint64(4);
        case {'int64','uint64','double'}
            elementSize = uint64(8);
        otherwise
            error('ERROR: Unsupported data type "%s"!', dataType);
    end
end

function dataType = ToInternalDataType(type)
    type = lower(reshape(type,[1 numel(type)]));
    switch(type)
        case {'bool','boolean','logical'}
            dataType = 'boolean';
        case {'char'}
            dataType = 'char';
        case {'i8','int8','int8_t'}
            dataType = 'int8';
        case {'u8','uint8','uint8_t'}
            dataType = 'uint8';
        case {'i16','int16','int16_t'}
            dataType = 'int16';
        case {'u16','uint16','uint16_t'}
            dataType = 'uint16';
        case {'i32','int32','int32_t','int'}
            dataType = 'int32';
        case {'u32','uint32','uint32_t','unsigned','unsigned int'}
            dataType = 'uint32';
        case {'i64','int64','int64_t'}
            dataType = 'int64';
        case {'u64','uint64','uint64_t','size_t'}
            dataType = 'uint64';
        case {'f32','float','float32','single'}
            dataType = 'single';
        case {'f64','double','float64'}
            dataType = 'double';
        otherwise
            error(['Unknown data-type "',type,'"!']);
    end
end
