function taps = write_fir_hex(coeffs, filename, varargin)
%WRITE_FIR_HEX Write FIR coefficients as a tap file for fir_decimator.
%   TAPS = WRITE_FIR_HEX(COEFFS, FILENAME) quantizes COEFFS to 18-bit two's
%   complement and writes them to FILENAME, one 5-digit hex word per line
%   (the rom's $readmemh format). Returns the integer taps written.
%
%   COEFFS(1) multiplies the newest sample, as in filter(), so a filter from
%   design() can be passed as d.Numerator. Set fir_decimator's TAPS to
%   numel(COEFFS) and INIT_FILE to the file (relative to the Efinity project).
%
%   fir_decimator outputs sum(taps .* x) / 2^18 (the top 18 bits of its
%   36-bit sum), so by default the taps are round(COEFFS * 2^18), which
%   keeps the filter's gain. Each coefficient then has to be under 0.5 in
%   magnitude (a decimate-by-4 lowpass peaks around 0.25).
%
%   Options (name/value):
%     'Normalize'  false (default), or true to scale so sum(abs(taps)) is
%                  2^18-1 instead: the most gain that no input can overflow.
%     'Width'      Tap and sample width (default 18).
%
%   Example:
%     write_fir_hex(d.Numerator, '../src/dsp/building_blocks/fir_taps.hex');

    p = inputParser;
    p.addParameter('Normalize', false);
    p.addParameter('Width', 18);
    p.parse(varargin{:});
    width = p.Results.Width;

    coeffs = double(coeffs(:));
    if p.Results.Normalize
        taps = round(coeffs * (2^width - 1) / sum(abs(coeffs)));
        % Rounding can push it just over
        while sum(abs(taps)) > 2^width - 1
            taps = taps - sign(taps) .* (abs(taps) == max(abs(taps)));
        end
    else
        taps = round(coeffs * 2^width);
    end

    lo = -2^(width-1);
    hi = 2^(width-1) - 1;
    if any(taps < lo | taps > hi)
        error('write_fir_hex:range', ...
              'Taps outside %d bits (largest %g): scale COEFFS down or use ''Normalize'', true', ...
              width, max(abs(taps)));
    end
    % The sum is 2*width bits: full-scale samples can overflow it if sum(abs(taps)) >= 2^width
    if sum(abs(taps)) >= 2^width
        warning('write_fir_hex:overflow', ...
                ['sum(abs(taps)) is %.3f x 2^%d: a full-scale input can overflow ' ...
                 'fir_decimator''s accumulator. ''Normalize'', true avoids it.'], ...
                sum(abs(taps)) / 2^width, width);
    end

    words = mod(taps, 2^width);  % two's complement
    fid = fopen(filename, 'w');
    if fid < 0
        error('write_fir_hex:open', 'Can''t open %s', filename);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, sprintf('%%0%dx\\n', ceil(width / 4)), words);

    fprintf('%s: %d taps, DC gain %.4f, sum(abs(taps)) = %.3f x 2^%d\n', ...
            filename, numel(taps), sum(taps) / 2^width, sum(abs(taps)) / 2^width, width);
end
