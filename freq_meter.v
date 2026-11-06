// =============================================================
// freq_ch - do tan so 1 kenh (trung binh theo cua so)
// =============================================================
module freq_ch #(
    parameter integer SAMPLE_RATE = 20000,          // mau/giay CUA 1 KENH
    parameter integer WINDOW      = 5000,           // so mau/cua so (~0.25 s)
    parameter [9:0]   MIN_RANGE   = 10'd16          // bien do toi thieu (LSB)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        s_valid,
    input  wire [9:0]  s_data,
    output reg  [31:0] freq_hz,
    output reg         freq_valid
);

    localparam integer TIMEOUT = 2 * SAMPLE_RATE;   // 2 s khong co canh -> 0 Hz

    // ---------------------------------------------------------
    // Nguong thich nghi: theo doi min/max moi WINDOW mau
    // ---------------------------------------------------------
    reg  [9:0]  mm_min, mm_max;
    reg  [9:0]  thr_hi, thr_lo;
    reg  [15:0] mm_cnt;
    reg         sig_ok;

    wire [9:0]  mm_range = mm_max - mm_min;
    wire [10:0] mm_sum   = mm_min + mm_max;
    wire [9:0]  mm_mid   = mm_sum[10:1];
    wire [9:0]  mm_hys   = {3'b000, mm_range[9:3]};   // range / 8

    // ---------------------------------------------------------
    // Do chu ky
    // ---------------------------------------------------------
    reg         above;
    reg         have_edge;
    reg  [31:0] sc;          // so mau tu canh bat dau cua so
    reg  [15:0] edge_cnt;    // so chu ky da hoan thanh trong cua so

    // ---------------------------------------------------------
    // Bo chia tuan tu 32 bit (restoring)
    // ---------------------------------------------------------
    reg         div_busy;
    reg  [5:0]  div_cnt;
    reg  [31:0] div_num, div_den, div_rem, div_quo;

    wire [31:0] shifted  = {div_rem[30:0], div_num[31]};
    wire        ge       = (shifted >= div_den);
    wire [31:0] quo_next = {div_quo[30:0], ge};


    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            freq_hz    <= 32'd0;
            freq_valid <= 1'b0;

            mm_min <= 10'd1023;
            mm_max <= 10'd0;
            thr_hi <= 10'd524;
            thr_lo <= 10'd500;
            mm_cnt <= 16'd0;
            sig_ok <= 1'b0;

            above     <= 1'b0;
            have_edge <= 1'b0;
            sc        <= 32'd0;
            edge_cnt  <= 16'd0;

            div_busy <= 1'b0;
            div_cnt  <= 6'd0;
            div_num  <= 32'd0;
            div_den  <= 32'd1;
            div_rem  <= 32'd0;
            div_quo  <= 32'd0;

        end

        else begin

            freq_valid <= 1'b0;

            // -------------------------------------------------
            // Bo chia: 1 bit / clk
            // -------------------------------------------------
            if (div_busy) begin

                div_rem <= ge ? (shifted - div_den) : shifted;
                div_quo <= quo_next;
                div_num <= {div_num[30:0], 1'b0};
                div_cnt <= div_cnt - 1'b1;

                if (div_cnt == 6'd1) begin
                    div_busy   <= 1'b0;
                    freq_hz    <= quo_next;
                    freq_valid <= 1'b1;
                end

            end

            // -------------------------------------------------
            // Moi mau hop le cua kenh nay
            // -------------------------------------------------
            if (s_valid) begin

                // ---- theo doi min/max, cap nhat nguong ----
                if (mm_cnt >= WINDOW - 1) begin

                    mm_cnt <= 16'd0;

                    if (mm_range >= MIN_RANGE) begin
                        thr_hi <= mm_mid + mm_hys;
                        thr_lo <= mm_mid - mm_hys;
                        sig_ok <= 1'b1;
                    end
                    else begin
                        sig_ok <= 1'b0;
                    end

                    mm_min <= 10'd1023;
                    mm_max <= 10'd0;

                end
                else begin

                    mm_cnt <= mm_cnt + 1'b1;

                    if (s_data < mm_min) mm_min <= s_data;
                    if (s_data > mm_max) mm_max <= s_data;

                end

                // ---- do tan so ----
                if (sc >= TIMEOUT) begin

                    // mat tin hieu qua lau
                    freq_hz    <= 32'd0;
                    freq_valid <= 1'b1;
                    have_edge  <= 1'b0;
                    above      <= 1'b0;
                    sc         <= 32'd0;
                    edge_cnt   <= 16'd0;

                end

                else if (sig_ok && !above && (s_data >= thr_hi)) begin

                    // canh len
                    above <= 1'b1;

                    if (!have_edge) begin

                        // canh dau tien: bat dau cua so
                        have_edge <= 1'b1;
                        sc        <= 32'd0;
                        edge_cnt  <= 16'd0;

                    end
                    else if ((sc + 32'd1) >= WINDOW) begin

                        // du cua so: freq = Fs * so_chu_ky / so_mau
                        if (!div_busy) begin
                            div_num  <= SAMPLE_RATE * ({16'd0, edge_cnt} + 32'd1);
                            div_den  <= sc + 32'd1;
                            div_rem  <= 32'd0;
                            div_quo  <= 32'd0;
                            div_cnt  <= 6'd32;
                            div_busy <= 1'b1;
                        end

                        sc       <= 32'd0;     // canh nay mo cua so moi
                        edge_cnt <= 16'd0;

                    end
                    else begin

                        sc       <= sc + 1'b1;
                        edge_cnt <= edge_cnt + 1'b1;

                    end

                end

                else begin

                    if (!sig_ok || (above && (s_data <= thr_lo)))
                        above <= 1'b0;

                    // dem MOI mau (ke ca mau roi xuong)
                    if (sc != 32'hFFFFFFFF)
                        sc <= sc + 1'b1;

                end

            end

        end

    end

endmodule


// =============================================================
// freq_meter - giu nguyen cong, top.v khong can sua
// =============================================================
module freq_meter #(
    parameter integer SAMPLE_RATE = 20000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [9:0]  adc_data,
    input  wire        adc_valid,
    input  wire        adc_ch,

    output wire [31:0] freq1_hz,
    output wire [31:0] freq2_hz,
    output wire        freq1_valid,
    output wire        freq2_valid
);

    freq_ch #(.SAMPLE_RATE(SAMPLE_RATE), .WINDOW(SAMPLE_RATE/4)) u_ch1 (
        .clk(clk), .rst_n(rst_n),
        .s_valid(adc_valid && !adc_ch),
        .s_data(adc_data),
        .freq_hz(freq1_hz), .freq_valid(freq1_valid)
    );

    freq_ch #(.SAMPLE_RATE(SAMPLE_RATE), .WINDOW(SAMPLE_RATE/4)) u_ch2 (
        .clk(clk), .rst_n(rst_n),
        .s_valid(adc_valid && adc_ch),
        .s_data(adc_data),
        .freq_hz(freq2_hz), .freq_valid(freq2_valid)
    );

endmodule