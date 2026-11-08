// Ethernet: streams the ADC samples, microphone samples and direction estimates out as
// RTP over UDP, through the RMII PHY. The PHY's management interface (eth_smi) is
// separate.
//
//   adc_axis -> CDC -> rtp_packetizer (ADC_SSRC) --\
//   mic_axis -> CDC -> rtp_packetizer (MIC_SSRC) ---> axis_packet_switch -> udp_stack -> rmii
//   dir_axis -> split x,y,z -> CDC -> rtp_packetizer (DIR_SSRC) --/            ^   |
//                                                                 udp_stack_regs (APB)
//
// Each source is one RTP stream of interleaved channels, told apart by SSRC; all go to
// the same UDP port. Sample streams must be in channel order (tid 0, 1, 2, ...).
// Directions are 3 channels, x, y and z, of DIR_BITS each (L16 for 16 bits).
//
// Packet size: RTP_PAYLOAD_BYTES is the most sample bytes per packet. The ADC and mic
// streams put as many whole frames (one sample of every channel) in a packet as fit,
// so their packets are always the same size:
//   frames per packet = floor(RTP_PAYLOAD_BYTES / (channels * bytes per sample))
// Use a multiple of both streams' frame sizes to fill packets exactly. It must fit in
// one datagram: RTP_PAYLOAD_BYTES <= 1472 - 12 = 1460. Directions are slow (one per
// correlation window), so their packets are DIR_FRAMES directions instead, to keep the
// latency down.
//
// It also sizes the buffers, which are most of this module's block RAM:
//   - the switch's per-stream packet buffers: 1.5 packets, rounded up to a power of 2
//     (the direction stream's for its own, small, packets)
//   - the TX frame buffer in udp_stack: one frame, rounded up to a power of 2
// The RX frame buffer is fixed (RX_AWIDTH), as frames from outside can be any size.
//
// TX_BUFFER = 0 leaves out the TX frame buffer. rmii_tx can't wait for data mid-frame,
// but it doesn't need to here: the switch only starts a packet once all of it is
// buffered, and sends it without gaps, so the packetizers after it never run dry.
//
// Received datagrams are only recorded in udp_stack_regs; their payload is thrown away.
`ifdef LINT_INCLUDES
`include "packetization/net_pkg.sv"
`endif
module ethernet
    import net_pkg::*;
#(
    parameter              RTP_PAYLOAD_BYTES = 1152,

    parameter              ADC_BITS          = 8,
    parameter              ADC_CHANNELS      = 8,
    parameter              ADC_ID_WIDTH      = 3,
    parameter logic [31:0] ADC_SSRC          = 32'h0000_0001,
    parameter logic [6:0]  ADC_PAYLOAD_TYPE  = 7'd96,

    parameter              MIC_BITS          = 18,
    parameter              MIC_CHANNELS      = 8,
    parameter              MIC_ID_WIDTH      = 3,
    parameter logic [31:0] MIC_SSRC          = 32'h0000_0002,
    parameter logic [6:0]  MIC_PAYLOAD_TYPE  = 7'd97,

    parameter              DIR_BITS          = 16,  // Each of x, y, z
    parameter              DIR_FRAMES        = 1,   // Directions per packet
    parameter logic [31:0] DIR_SSRC          = 32'h0000_0003,
    parameter logic [6:0]  DIR_PAYLOAD_TYPE  = 7'd98,

    parameter              CDC_AWIDTH        = 4,   // Sample CDC FIFOs, 2^CDC_AWIDTH samples
    parameter              TX_BUFFER         = 1,   // TX frame buffer in udp_stack (see above)
    parameter              RX_AWIDTH         = 12   // RX frame buffer, 2^RX_AWIDTH bytes
)(
    input  logic        clk,        // 50 MHz, the RMII reference clock
    input  logic        rstn,
    input  logic        clk_io,     // Clock of the sample streams
    input  logic        rstn_io,
    input  logic        clk_dir,    // Clock of the direction stream
    input  logic        rstn_dir,

    // RMII PHY
    output logic [1:0]  rmii_txd,
    output logic        rmii_txen,
    output logic        rmii_rstn,
    input  logic        rmii_int,
    input  logic        rmii_crs_dv,
    input  logic [1:0]  rmii_rxd,
    input  logic        rmii_rxerr,

    apb_if.slv          apb,        // udp_stack_regs, on clk. Register map in udp_stack_regs.sv

    // Samples, on clk_io
    axis_tid_if.slv     adc_axis,
    axis_tid_if.slv     mic_axis,

    // Directions {x, y, z}, DIR_BITS*3 wide, on clk_dir
    axis_if.slv         dir_axis
);

    // ------------------------------------------------------------------
    // Sizes
    // ------------------------------------------------------------------
    localparam DIR_CHANNELS     = 3; // x, y, z
    localparam DIR_ID_WIDTH     = 2;

    localparam ADC_SAMPLE_BYTES = (ADC_BITS + 7) / 8;
    localparam MIC_SAMPLE_BYTES = (MIC_BITS + 7) / 8;
    localparam DIR_SAMPLE_BYTES = (DIR_BITS + 7) / 8;
    localparam ADC_FRAMES       = RTP_PAYLOAD_BYTES / (ADC_CHANNELS * ADC_SAMPLE_BYTES);
    localparam MIC_FRAMES       = RTP_PAYLOAD_BYTES / (MIC_CHANNELS * MIC_SAMPLE_BYTES);

    // RTP packet (= UDP payload) bytes
    localparam ADC_RTP_BYTES    = RTP_HDR_BYTES + ADC_FRAMES * ADC_CHANNELS * ADC_SAMPLE_BYTES;
    localparam MIC_RTP_BYTES    = RTP_HDR_BYTES + MIC_FRAMES * MIC_CHANNELS * MIC_SAMPLE_BYTES;
    localparam DIR_RTP_BYTES    = RTP_HDR_BYTES + DIR_FRAMES * DIR_CHANNELS * DIR_SAMPLE_BYTES;
    localparam MAX_RTP_BYTES    = ADC_RTP_BYTES > MIC_RTP_BYTES ? ADC_RTP_BYTES : MIC_RTP_BYTES;

    // Buffers. Frames shorter than the minimum (small direction packets) are padded
    // by eth_packetizer.
    localparam SWITCH_AWIDTH    = $clog2(MAX_RTP_BYTES + MAX_RTP_BYTES / 2);
    localparam DIR_AWIDTH       = $clog2(DIR_RTP_BYTES + DIR_RTP_BYTES / 2);
    localparam TX_FRAME_BYTES   = ETH_HDR_BYTES + IPV4_HDR_BYTES + UDP_HDR_BYTES + MAX_RTP_BYTES + ETH_FCS_BYTES;
    localparam TX_AWIDTH        = $clog2(TX_FRAME_BYTES);

    // Switch inputs
    localparam ADC_STREAM = 0;
    localparam MIC_STREAM = 1;
    localparam DIR_STREAM = 2;


    // ------------------------------------------------------------------
    // Samples to clk
    // ------------------------------------------------------------------
    axis_tid_if #(.WIDTH(ADC_BITS), .ID_WIDTH(ADC_ID_WIDTH)) adc_samples ();
    axis_tid_if #(.WIDTH(MIC_BITS), .ID_WIDTH(MIC_ID_WIDTH)) mic_samples ();

    axis_tid_cdc_fifo #(
        .WIDTH(ADC_BITS),
        .ID_WIDTH(ADC_ID_WIDTH),
        .AWIDTH(CDC_AWIDTH)
    ) adc_cdc (
        .clka(clk_io),
        .rstna(rstn_io),
        .clkb(clk),
        .rstnb(rstn),
        .axis_in(adc_axis),
        .axis_out(adc_samples)
    );

    axis_tid_cdc_fifo #(
        .WIDTH(MIC_BITS),
        .ID_WIDTH(MIC_ID_WIDTH),
        .AWIDTH(CDC_AWIDTH)
    ) mic_cdc (
        .clka(clk_io),
        .rstna(rstn_io),
        .clkb(clk),
        .rstnb(rstn),
        .axis_in(mic_axis),
        .axis_out(mic_samples)
    );

    // Directions are split into x, y, z first, so the CDC FIFO is one component wide
    axis_tid_if #(.WIDTH(DIR_BITS), .ID_WIDTH(DIR_ID_WIDTH)) dir_components ();
    axis_tid_if #(.WIDTH(DIR_BITS), .ID_WIDTH(DIR_ID_WIDTH)) dir_samples ();

    axis_tid_serializer #(
        .WIDTH(DIR_BITS),
        .N(DIR_CHANNELS),
        .ID_WIDTH(DIR_ID_WIDTH)
    ) dir_split (
        .clk(clk_dir),
        .rstn(rstn_dir),
        .axis_in(dir_axis),
        .axis_out(dir_components)
    );

    axis_tid_cdc_fifo #(
        .WIDTH(DIR_BITS),
        .ID_WIDTH(DIR_ID_WIDTH),
        .AWIDTH(CDC_AWIDTH)
    ) dir_cdc (
        .clka(clk_dir),
        .rstna(rstn_dir),
        .clkb(clk),
        .rstnb(rstn),
        .axis_in(dir_components),
        .axis_out(dir_samples)
    );


    // ------------------------------------------------------------------
    // RTP, one stream per source, merged a packet at a time
    // ------------------------------------------------------------------
    axis_tlast_if #(.WIDTH(8)) rtp_streams [3] ();
    axis_tlast_if #(.WIDTH(8)) rtp_tx ();
    logic [1:0] rtp_tx_sel;

    rtp_packetizer #(
        .SSRC(ADC_SSRC),
        .PAYLOAD_TYPE(ADC_PAYLOAD_TYPE),
        .SAMPLE_BITS(ADC_BITS),
        .CHANNELS(ADC_CHANNELS),
        .ID_WIDTH(ADC_ID_WIDTH),
        .FRAMES_PER_PACKET(ADC_FRAMES)
    ) adc_rtp (
        .clk(clk),
        .rstn(rstn),
        .axis_in(adc_samples),
        .axis_out(rtp_streams[ADC_STREAM])
    );

    rtp_packetizer #(
        .SSRC(MIC_SSRC),
        .PAYLOAD_TYPE(MIC_PAYLOAD_TYPE),
        .SAMPLE_BITS(MIC_BITS),
        .CHANNELS(MIC_CHANNELS),
        .ID_WIDTH(MIC_ID_WIDTH),
        .FRAMES_PER_PACKET(MIC_FRAMES)
    ) mic_rtp (
        .clk(clk),
        .rstn(rstn),
        .axis_in(mic_samples),
        .axis_out(rtp_streams[MIC_STREAM])
    );

    rtp_packetizer #(
        .SSRC(DIR_SSRC),
        .PAYLOAD_TYPE(DIR_PAYLOAD_TYPE),
        .SAMPLE_BITS(DIR_BITS),
        .CHANNELS(DIR_CHANNELS),
        .ID_WIDTH(DIR_ID_WIDTH),
        .FRAMES_PER_PACKET(DIR_FRAMES)
    ) dir_rtp (
        .clk(clk),
        .rstn(rstn),
        .axis_in(dir_samples),
        .axis_out(rtp_streams[DIR_STREAM])
    );

    axis_packet_switch #(
        .WIDTH(8),
        .N(3),
        .AWIDTH(SWITCH_AWIDTH),
        .AWIDTHS({8'(DIR_AWIDTH), 8'd0, 8'd0}) // The direction stream's buffer for its own packets
    ) rtp_switch (
        .clk(clk),
        .rstn(rstn),
        .axis_in(rtp_streams),
        .axis_out(rtp_tx),
        .out_sel(rtp_tx_sel)
    );

    // udp_stack cuts datagrams by length, so tlast isn't passed on. The length is the
    // size of the stream's packets.
    axis_if #(.WIDTH(8)) udp_tx_payload ();
    logic [15:0]         udp_tx_length;

    assign udp_tx_payload.tvalid = rtp_tx.tvalid;
    assign udp_tx_payload.tdata  = rtp_tx.tdata;
    assign rtp_tx.tready         = udp_tx_payload.tready;
    assign udp_tx_length         = rtp_tx_sel == MIC_STREAM ? 16'(MIC_RTP_BYTES) :
                                   rtp_tx_sel == DIR_STREAM ? 16'(DIR_RTP_BYTES) : 16'(ADC_RTP_BYTES);


    // ------------------------------------------------------------------
    // UDP/IPv4 stack and its registers
    // ------------------------------------------------------------------
    axis_tlast_if #(.WIDTH(8)) mac_tx ();
    axis_tlast_if #(.WIDTH(8)) mac_rx ();

    logic [47:0] local_mac;
    logic [31:0] local_ip;
    logic        promiscuous;

    eth_hdr_if  tx_eth_hdr ();
    ipv4_hdr_if tx_ip_hdr ();
    udp_hdr_if  tx_udp_hdr ();
    eth_hdr_if  rx_eth_hdr ();
    ipv4_hdr_if rx_ip_hdr ();
    udp_hdr_if  rx_udp_hdr ();
    axis_tlast_if #(.WIDTH(8)) udp_rx_payload ();

    udp_stack #(
        .TX_BUFFER(TX_BUFFER),
        .TX_AWIDTH(TX_AWIDTH),
        .RX_AWIDTH(RX_AWIDTH)
    ) udp_stack_i (
        .clk        (clk),
        .rstn       (rstn),
        .local_mac  (local_mac),
        .local_ip   (local_ip),
        .promiscuous(promiscuous),
        .mac_tx     (mac_tx),
        .mac_rx     (mac_rx),
        .tx_eth_hdr (tx_eth_hdr),
        .tx_ip_hdr  (tx_ip_hdr),
        .tx_udp_hdr (tx_udp_hdr),
        .tx_payload (udp_tx_payload),
        .rx_eth_hdr (rx_eth_hdr),
        .rx_ip_hdr  (rx_ip_hdr),
        .rx_udp_hdr (rx_udp_hdr),
        .rx_payload (udp_rx_payload)
    );

    // Nothing uses received datagrams yet, beyond the registers
    assign udp_rx_payload.tready = 1'b1;
    assign rx_udp_hdr.ready      = 1'b1;
    assign rx_eth_hdr.ready      = 1'b1; // Ignored by udp_stack
    assign rx_ip_hdr.ready       = 1'b1; // Ignored by udp_stack

    udp_stack_regs udp_stack_regs_i (
        .clk        (clk),
        .rstn       (rstn),
        .apb        (apb),
        .local_mac  (local_mac),
        .local_ip   (local_ip),
        .promiscuous(promiscuous),
        .tx_length  (udp_tx_length),
        .tx_eth_hdr (tx_eth_hdr),
        .tx_ip_hdr  (tx_ip_hdr),
        .tx_udp_hdr (tx_udp_hdr),
        .rx_eth_hdr (rx_eth_hdr),
        .rx_ip_hdr  (rx_ip_hdr),
        .rx_udp_hdr (rx_udp_hdr)
    );


    // ------------------------------------------------------------------
    // RMII MAC
    // ------------------------------------------------------------------
    rmii rmii_i (
        .clk        (clk),
        .rstn       (rstn),
        .rmii_txd   (rmii_txd),
        .rmii_txen  (rmii_txen),
        .rmii_rstn  (rmii_rstn),
        .rmii_int   (rmii_int),
        .rmii_crs_dv(rmii_crs_dv),
        .rmii_rxd   (rmii_rxd),
        .rmii_rxerr (rmii_rxerr),
        .axis_in    (mac_tx),
        .axis_out   (mac_rx)
    );

endmodule
