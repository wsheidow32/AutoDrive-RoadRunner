% Exporting Scenario and Reading
exportScenario(rrApp, fileNameWithOutExt, "OpenSCENARIO")
xoscPath = fullfile(rrProjectPath_Final, "Exports", fileNameWithOutExt + ".xosc");
ExportedFile = fileread(xoscPath);

% Checking if Car Straight Scenario
if contains(fileNameWithOutExt, "NoObstacle")
    if contains(fileNameWithOutExt, "Straight") || contains(fileNameWithOutExt, "Right")
        newX = randi([-50 0]);
        replacement = sprintf(['<Private entityRef="Sedan2">\n\t\t' ...
            '<PrivateAction>\n\t\t\t\t\t<TeleportAction>\n\t\t\t\t\t' ...
            '<Position>\n\t\t\t\t\t<WorldPosition h="3.14159" p="0" r="0" x="%f" y="1.43420004844666" z="0.000000"/>\n\t\t\t\t\t\t\t'...
            '</Position>'], newX);
        pattern = ['<Private entityRef="Sedan2">(.*?)<PrivateAction>(.*?)<TeleportAction>(.*?)<Position>(.*?)<WorldPosition(.*?)/>(.*?)</Position>';];
        FileWithRandom = regexprep(ExportedFile, pattern, replacement);
    else
       newY = randi([0 50]);
        replacement = sprintf(['<Private entityRef="Sedan2">\n\t\t' ...
            '<PrivateAction>\n\t\t\t\t\t<TeleportAction>\n\t\t\t\t\t' ...
            '<Position>\n\t\t\t\t\t<WorldPosition h="1.5708" p="0" r="0" x="2.754790" y="%f" z="0.066688"/>\n\t\t\t\t\t\t\t'...
            '</Position>'], newY);
        pattern = ['<Private entityRef="Sedan2">(.*?)<PrivateAction>(.*?)<TeleportAction>(.*?)<Position>(.*?)<WorldPosition(.*?)/>(.*?)</Position>';];
        FileWithRandom = regexprep(ExportedFile, pattern, replacement); 
    end
end

% Checking if Parallel
if contains(fileNameWithOutExt, "Parrallel")
        newY = randi([-50 70]);
        replacement = sprintf(['<Private entityRef="Sedan2">\n\t\t' ...
            '<PrivateAction>\n\t\t\t\t\t<TeleportAction>\n\t\t\t\t\t' ...
            '<Position>\n\t\t\t\t\t<WorldPosition h="-1.5708" p="0" r="0" x="-1.297150" y="%f" z="0.000000"/>\n\t\t\t\t\t\t\t'...
            '</Position>'], newY);
        pattern = ['<Private entityRef="Sedan2">(.*?)<PrivateAction>(.*?)<TeleportAction>(.*?)<Position>(.*?)<WorldPosition(.*?)/>(.*?)</Position>';];
        FileWithRandom = regexprep(ExportedFile, pattern, replacement);
end

% Checking if Perpendicular
if contains(fileNameWithOutExt, "Perpendicular")
        newX = randi([-80 10]);
        replacement = sprintf(['<Private entityRef="Sedan2">\n\t\t' ...
            '<PrivateAction>\n\t\t\t\t\t<TeleportAction>\n\t\t\t\t\t' ...
            '<Position>\n\t\t\t\t\t<WorldPosition h="0" p="0" r="0" x="%f" y="-2.130000" z="0.000000"/>\n\t\t\t\t\t\t\t'...
            '</Position>'], newX);
        pattern = ['<Private entityRef="Sedan2">(.*?)<PrivateAction>(.*?)<TeleportAction>(.*?)<Position>(.*?)<WorldPosition(.*?)/>(.*?)</Position>';];
        FileWithRandom = regexprep(ExportedFile, pattern, replacement);
end

% Writing New Coordinates into File
fid = fopen(xoscPath, 'w');
fwrite(fid, FileWithRandom);
fclose(fid);

% Copying and Moving files into Assets Folder
assetsPath  = fullfile(rrProjectPath_Final,"Assets", fileNameWithOutExt + ".xosc");
exportsDir = fullfile(rrProjectPath_Final, "Exports");
assetsDir  = fullfile(rrProjectPath_Final, "Assets");
copyfile(xoscPath, assetsPath, 'f');
copyfile(fullfile(exportsDir, fileNameWithOutExt + ".xodr"), assetsDir, 'f');
copyfile(fullfile(exportsDir, fileNameWithOutExt + ".xosc"), assetsDir, 'f');
copyfile(fullfile(exportsDir, fileNameWithOutExt + ".osgb"), assetsDir, 'f');


% Closing RAG Scenario
close(rrApp)

% Opening Blank Scenario
rrApp = roadrunner(rrProjectPath_Final);
fileNameWithOutExt = erase(fileNameWithOutExt,"_RAG")
openScenario(rrApp, [fileNameWithOutExt '_Blank.rrscenario'])

% Importing the new Scenario
importScenario(rrApp, [fileNameWithOutExt '_RAG.xosc'], "OpenSCENARIO");