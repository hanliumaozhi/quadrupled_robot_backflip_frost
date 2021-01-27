function [rootPath] = get_root_path()
    pathstr = mfilename('fullpath');    % mfilename() : File name of currently running code
    [pathstr, ~, ~] = fileparts([pathstr, '.m']);
    [rootPath, ~, ~] = fileparts(pathstr);
end
