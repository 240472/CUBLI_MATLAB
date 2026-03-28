
%% Clear contents of a folder but keep the folder

foldersToClean = {'work\cache', 'work\codegen', 'cache', 'codegen'};    % Must be from project root directory

for i = 1:numel(foldersToClean)
    targetFolder = fullfile(pwd, foldersToClean(i));  % Change 'work' to your folder name
    
    if isfolder(targetFolder{1})
        
        % Delete all subfolders recursively
        subfolders = dir(targetFolder{1});
        subfolders = subfolders([subfolders.isdir]); % keep only directories
        for k = 1:numel(subfolders)
            name = subfolders(k).name;
            if ~strcmp(name, '.') && ~strcmp(name, '..')
                rmdir(fullfile(targetFolder{1}, name), 's');
            end
        end
        
        % Delete all files for cache folder
        if strcmp(foldersToClean(i), 'work\cache') | strcmp(foldersToClean(i), 'cache')
            delete(fullfile(targetFolder{1}, '*'));
        end
    end
end