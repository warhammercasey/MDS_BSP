// APB clock domain crossing bridge: a peripheral in clkb behind an APB master in clka.
//
//   apb_a (clka) -> slave port, connect to the master side (e.g. an apb_splitter output)
//   apb_b (clkb) -> master port, connect to the peripheral
//
// Each transfer is handed across with a toggle handshake:
//   1. clka registers paddr/pwrite/pwdata in the setup phase and flips req_tgl.
//   2. clkb sees req_tgl differ from its ack_tgl, registers the request, and runs the
//      transfer on apb_b.
//   3. clkb registers prdata/pslverr and flips ack_tgl.
//   4. clka sees ack_tgl match req_tgl again and raises pready, with prdata/pslverr
//      taken straight from the clkb registers.
// Only the two toggles go through synchronizers. The request and response registers
// are held still from before their toggle flips until after the other side has used
// them, so they cross without one (they are false paths for timing).
//
// A transfer is outstanding while the toggles differ. The toggles have no reset (they
// power up 0), so the two sides agree on that whichever domain leaves reset first, or
// if only one is reset. A request made while clkb is still in reset runs once clkb
// leaves reset.
//
// If the master drops psel before pready (uart_controller's timeout does this), the
// transfer still completes on apb_b and its response is thrown away. The next
// transfer waits for it. A peripheral that never raises pready blocks the bridge for
// good, so every later transfer through it times out.
//
// Latency: about 3 clkb + 3 clka cycles on top of the peripheral's own transfer.
module apb_cdc #(
    parameter ADDR_WIDTH = 8,
    parameter DATA_WIDTH = 8
) (
    input  logic clka,
    input  logic rstna,
    apb_if.slv   apb_a,   // SEL_WIDTH = 1

    input  logic clkb,
    input  logic rstnb,
    apb_if.mst   apb_b    // SEL_WIDTH = 1
);

    // Handshake toggles. No reset: see above.
    logic req_tgl = 1'b0;  // clka
    logic ack_tgl = 1'b0;  // clkb
    logic req_tgl_b;       // req_tgl synchronized to clkb
    logic ack_tgl_a;       // ack_tgl synchronized to clka

    // Request (clka) and response (clkb) registers, read by the other domain
    logic [ADDR_WIDTH-1:0] req_addr;
    logic                  req_write;
    logic [DATA_WIDTH-1:0] req_wdata;
    logic [DATA_WIDTH-1:0] resp_rdata;
    logic                  resp_slverr;

    ff_sync #(
        .WIDTH(1)
    ) ff_sync_req (
        .clka(clka),
        .clkb(clkb),
        .din(req_tgl),
        .dout(req_tgl_b)
    );

    ff_sync #(
        .WIDTH(1)
    ) ff_sync_ack (
        .clka(clkb),
        .clkb(clka),
        .din(ack_tgl),
        .dout(ack_tgl_a)
    );

    // ------------------------------------------------------------------
    // clka: take the master's transfer, return the response
    // ------------------------------------------------------------------
    logic pending_a;    // A request is in flight
    logic pending_a_q;
    logic issued;       // The master's current transfer has sent its request
    logic issue;
    logic done_a;       // The request in flight just completed

    assign pending_a = req_tgl != ack_tgl_a;
    assign done_a    = pending_a_q && !pending_a;
    // Send once per transfer, after any abandoned transfer before it has completed
    assign issue     = apb_a.psel[0] && !issued && !pending_a;

    assign apb_a.pready  = apb_a.psel[0] && apb_a.penable && issued && done_a;
    assign apb_a.prdata  = resp_rdata;
    assign apb_a.pslverr = resp_slverr;

    always_ff @(posedge clka) begin
        if (issue) begin
            req_tgl   <= ~req_tgl;
            req_addr  <= apb_a.paddr;
            req_write <= apb_a.pwrite;
            req_wdata <= apb_a.pwdata;
        end
    end

    always_ff @(posedge clka) begin
        if (~rstna) begin
            pending_a_q <= 1'b0;
            issued      <= 1'b0;
        end else begin
            pending_a_q <= pending_a;
            // The transfer ends with pready, or is abandoned when psel drops
            if (!apb_a.psel[0] || apb_a.pready)
                issued <= 1'b0;
            else if (issue)
                issued <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // clkb: run the transfer on the peripheral's bus
    // ------------------------------------------------------------------
    typedef enum logic [1:0] {
        B_IDLE,
        B_SETUP,
        B_ACCESS
    } state_b_t;

    state_b_t              state_b;
    logic [ADDR_WIDTH-1:0] addr_b;
    logic                  write_b;
    logic [DATA_WIDTH-1:0] wdata_b;

    always_ff @(posedge clkb) begin
        if (~rstnb) begin
            state_b <= B_IDLE;
        end else begin
            case (state_b)
                B_IDLE: begin
                    if (req_tgl_b != ack_tgl) begin
                        addr_b  <= req_addr;
                        write_b <= req_write;
                        wdata_b <= req_wdata;
                        state_b <= B_SETUP;
                    end
                end

                B_SETUP: state_b <= B_ACCESS;

                B_ACCESS: begin
                    if (apb_b.pready) begin
                        resp_rdata  <= apb_b.prdata;
                        resp_slverr <= apb_b.pslverr;
                        ack_tgl     <= ~ack_tgl;
                        state_b     <= B_IDLE;
                    end
                end

                default: state_b <= B_IDLE;
            endcase
        end
    end

    assign apb_b.psel[0] = state_b != B_IDLE;
    assign apb_b.penable = state_b == B_ACCESS;
    assign apb_b.paddr   = addr_b;
    assign apb_b.pwrite  = write_b;
    assign apb_b.pwdata  = wdata_b;

endmodule
