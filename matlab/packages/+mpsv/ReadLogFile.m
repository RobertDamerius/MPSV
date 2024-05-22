function log = ReadLogFile(filename)
    %mpsv.ReadLogFile Read and decode a log file.
    % 
    % PARAMETER
    % filename   ... Name of the logfile.
    assert(ischar(filename), 'ERROR: Input "filename" must be a string!');
    file = fopen(filename,'r');
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
            assert(fieldSize >= uint64(numberOfDimensions * 4), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(numberOfDimensions * 4);
            [V,L] = fread(file,numberOfDimensions,'uint32'); assert(L == numberOfDimensions, 'ERROR: Invalid field data format!');
            dimension = uint32(V)';

            % Value
            assert(fieldSize >= uint64(8), 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - uint64(8);
            [V,L] = fread(file,1,'uint64'); assert(L == 1, 'ERROR: Invalid field data format!');
            sizeofValue = uint64(V);
            assert(fieldSize >= sizeofValue, 'ERROR: Invalid field data format!');
            fieldSize = fieldSize - sizeofValue;
            if(strcmp('boolean',dataType))
                [V,L] = fread(file,dimension,'uint8'); assert(L == prod(dimension), 'ERROR: Invalid field data format!');
                eval(['log{logNumber}.',name,' = ' dataType '(V > 0);']);
            else
                [V,L] = fread(file,dimension,dataType); assert(L == prod(dimension), 'ERROR: Invalid field data format!');
                eval(['log{logNumber}.',name,' = ' dataType '(V);']);
            end

            % Unknown field data
            if(fieldSize)
                fread(file,fieldSize,'uint8');
            end
        end
    end
    fclose(file);
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

