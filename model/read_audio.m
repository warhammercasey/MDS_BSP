function out = read_audio(folder)
recording_path = ['recordings/', folder, '/'];
recordingFiles = dir(fullfile(recording_path, '*.wav'));
assert(~isempty(recordingFiles), 'No MAT recordings found.');

recordingNames = {recordingFiles.name};
[~, sortOrder] = sort(recordingNames);
recordingFiles = recordingFiles(sortOrder);

channels = length(recordingFiles);
out = audioread([recording_path, recordingFiles(1).name]);
for i=2:channels
    new_channel = audioread([recording_path, recordingFiles(i).name]);
    if length(new_channel) > length(out)
        new_channel = new_channel(1:length(out));
    elseif length(out) > length(new_channel)
        out = out(1:length(new_channel), :);
    end
    out = [out, new_channel];
end
end