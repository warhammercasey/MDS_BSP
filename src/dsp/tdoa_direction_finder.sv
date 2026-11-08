// Estimates direction of input audio signals using Time Difference of Arrival (TDOA) method
// Pipeline: I2S Mics -> Mix to baseband -> Decimate by 4 -> Cross-correlation across each pair -> least-squares direction estimation
module tdoa_direction_finder #(
    // Global parameters
    parameter WIDTH    = 18, // Sample width
    parameter CHANNELS = 8,
    
    // Mixer parameters
    parameter SAMPLE_RATE     = 48_000, // Input sample rate
    parameter TONE_FREQ       = 5_000, // Frequency to mix down
    parameter PHASE_WIDTH     = 32, // Phase accumulator width
    parameter LUT_AWIDTH      = 8, // LUT address width
    parameter MIXER_INIT_FILE = "",
    parameter MIXER_INIT_HEX  = 1,

    // Decimation parameters
    parameter TAPS            = 32,
    parameter DECIMATION      = 4,
    parameter DECIM_INIT_FILE = "",
    parameter DECIM_INIT_HEX  = 1,

    // Cross-correlation parameters
    parameter WINDOW_LEN      = 2560,
    parameter WINDOW_STEP     = 32,
    parameter MAX_LAG         = 10,
    parameter XCORR_MACS      = 3,
    parameter XCORR_OUT_WIDTH = 32,
    
    // Least-squares direction estimation parameters
    parameter DF_COEF_FILE    = "",
    parameter DF_COEF_FRAC    = 23, // Must match gen_direction_matrix.py --coef-frac
    parameter DF_OUT_WIDTH    = 16,
    parameter DF_OUT_FRAC     = 14,

    // Moving average filter
    parameter MOVAVG_LOG2_WINDOW_SIZE = 4
) (
    input  logic clk,
    input  logic rstn,

    axis_tid_if.slv din,
    axis_if.mst dout
);

    // Complex from here to the correlator: {I, Q}, WIDTH each
    axis_tid_if #(
        .WIDTH(2*WIDTH),
        .ID_WIDTH($clog2(CHANNELS))
    ) mixer_out_if ();

    mixer #(
        .WIDTH(WIDTH),
        .TID_WIDTH($clog2(CHANNELS)),
        .SAMPLE_RATE(SAMPLE_RATE),
        .TONE_FREQ(TONE_FREQ),
        .PHASE_WIDTH(PHASE_WIDTH),
        .LUT_AWIDTH(LUT_AWIDTH),
        .INIT_FILE(MIXER_INIT_FILE),
        .INIT_HEX(MIXER_INIT_HEX)
    ) mixer_i (
        .clk(clk),
        .rstn(rstn),
        .din(din),
        .dout(mixer_out_if)
    );


    axis_tid_if #(
        .WIDTH(2*WIDTH),
        .ID_WIDTH($clog2(CHANNELS))
    ) decim_out_if ();

    fir_decimator #(
        .WIDTH(WIDTH),
        .TID_WIDTH($clog2(CHANNELS)),
        .TAPS(TAPS),
        .DECIMATION(DECIMATION),
        .INIT_FILE(DECIM_INIT_FILE),
        .INIT_HEX(DECIM_INIT_HEX)
    ) decimator_i (
        .clk(clk),
        .rstn(rstn),
        .din(mixer_out_if),
        .dout(decim_out_if)
    );

    localparam XCORR_TID_WIDTH = $clog2(CHANNELS*(CHANNELS-1)/2);
    axis_tid_tlast_if #(
        .WIDTH(2*XCORR_OUT_WIDTH), // {re, im}
        .ID_WIDTH(XCORR_TID_WIDTH)
    ) xcorr_out_if ();

    cross_correlator #(
        .WIDTH(WIDTH),
        .CHANNELS(CHANNELS),
        .WINDOW_LEN(WINDOW_LEN),
        .WINDOW_STEP(WINDOW_STEP),
        .MAX_LAG(MAX_LAG),
        .MACS(XCORR_MACS),
        .OUT_WIDTH(XCORR_OUT_WIDTH)
    ) cross_correlator_i (
        .clk(clk),
        .rstn(rstn),
        .din(decim_out_if),
        .dout(xcorr_out_if)
    );


    localparam XCORR_OUT_PACKET_WIDTH = 2*MAX_LAG + 1;
    axis_tid_if #(
        .WIDTH($clog2(XCORR_OUT_PACKET_WIDTH)),
        .ID_WIDTH(XCORR_TID_WIDTH)
    ) peak_finder_out_if ();

    peak_finder #(
        .WIDTH(XCORR_OUT_WIDTH),
        .MAX_LEN(XCORR_OUT_PACKET_WIDTH),
        .TID_WIDTH(XCORR_TID_WIDTH)
    ) peak_finder_i (
        .clk(clk),
        .rstn(rstn),
        .din(xcorr_out_if),
        .dout(peak_finder_out_if)
    );

    axis_if #(
        .WIDTH(DF_OUT_WIDTH*3)
    ) direction_estimator_out_if ();

    direction_estimator #(
        .CHANNELS(CHANNELS),
        .MAX_LAG(MAX_LAG),
        .COEF_FILE(DF_COEF_FILE),
        .COEF_FRAC(DF_COEF_FRAC),
        .OUT_WIDTH(DF_OUT_WIDTH),
        .OUT_FRAC(DF_OUT_FRAC)
    ) direction_finder_i (
        .clk(clk),
        .rstn(rstn),
        .din(peak_finder_out_if),
        .dout(direction_estimator_out_if)
    );

    moving_average_filter #(
        .LOG2_WINDOW_SIZE(MOVAVG_LOG2_WINDOW_SIZE),
        .DATA_WIDTH(DF_OUT_WIDTH),
        .NUM_CHANNELS(3)
    ) moving_average_filter_i (
        .clk(clk),
        .rstn(rstn),
        .din(direction_estimator_out_if),
        .dout(dout)
    );

endmodule