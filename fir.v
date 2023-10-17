module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(   // ---------------------------------------------------------------------------
    // AXI4-Lite Write Transaction   
    // 1. Master puts an address on awaddr and data on wdata. At the same time it asserts awvalid and wvalid.
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     awvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    input   wire                     wvalid,
    // 2.  Slave asserts awready and wready
    output  wire                     awready,
    output  wire                     wready,
    // ---------------------------------------------------------------------------
    // AXI4-Lite Read Transaction
    // 1. Master asserts arvalid, rready
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     arvalid,
    input   wire                     rready,
    // 2. Slave asserts arready
    output  wire                     arready,
    // 3. After handshake, Slave puts the requested data on rdata and asserts rvalid.
    output  wire [(pDATA_WIDTH-1):0] rdata, 
    output  wire                     rvalid,
    // ---------------------------------------------------------------------------
    // AXI4-Stream Transfer Protocol
    // 1. Slave side
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    input   wire                     ss_tvalid,
    output  reg                      ss_tready, 
    // 2. Master side
    input   wire                     sm_tready,  
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                     sm_tlast, 
    output  reg                     sm_tvalid, 
    // ---------------------------------------------------------------------------
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,
    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,
    // ---------------------------------------------------------------------------
    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
    // Wires and Registers
    // ---------------------------------------------------------------------------
    // ---- Add your own wires and registers here if needed ---- //
    // FSM state
    localparam IDLE     =  3'd0;
    localparam SET_UP   =  3'd1;
    localparam COMPUTE  =  3'd2;
    localparam DONE     =  3'd3;
    reg [2:0] state, state_next;
    // Program data
    reg [(pDATA_WIDTH-1):0] config_r;
    reg [(pDATA_WIDTH-1):0] data_length;
    // Write
    reg  write_en, write_en_next;
    reg  [(pADDR_WIDTH-1):0] write_addr_w, write_addr_r;
    reg  [(pDATA_WIDTH-1):0] write_data_w, write_data_r;
    reg  write_addr_received_w, write_addr_received_r;
    reg  write_data_received_w, write_data_received_r;
    // Read
    reg  [(pADDR_WIDTH-1):0] read_addr_w, read_addr_r;
    reg  [(pDATA_WIDTH-1):0] read_data_r;
    reg  [(pDATA_WIDTH-1):0] tap_ram_data_w, tap_ram_data_r;
    reg  read_addr_received_w, read_addr_received_r;
    reg  read_data_length_w, read_data_length_r;
    reg  read_config_w, read_config_r;
    reg  read_tap_ram_w, read_tap_ram_r;
    reg  read_valid_w, read_valid_r;

    reg [pADDR_WIDTH-1 : 0] tap_A_w, tap_A_r;
    reg first_data;
    reg [5:0] shift_counter;
    wire data_write_en;
    reg [1:0] data_read_stage;
    reg computing, outputing;
    reg last_data;
    reg signed [(pDATA_WIDTH-1):0] data_received;
    reg [9:0] data_counter;

    reg signed [(pDATA_WIDTH-1):0] accumulated_result;
    wire signed [(pDATA_WIDTH-1):0] mult_result;
    reg signed [(pDATA_WIDTH-1):0] mult1;
    wire signed [(pDATA_WIDTH-1):0] mult2;
    reg signed [(pDATA_WIDTH-1):0] current_data;
    // ---------------------------------------------------------------------------
    always @(*) begin
        case (state)
            IDLE: state_next = config_r[0] ? SET_UP : IDLE;
            SET_UP:  begin
                state_next = ss_tvalid ? COMPUTE : SET_UP;
            end
            COMPUTE: begin
                state_next = !(sm_tlast && sm_tready && sm_tvalid) ? COMPUTE : IDLE;
            end
            default: state_next = IDLE; 
        endcase
    end
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n)
            state <= IDLE;
        else
            state <= state_next;   
    end
    // ---------------------------------------------------------------------------
    // AXI4-Lite Write Transaction  
    
    // assign write_en = write_addr_received_r && write_data_received_r;
    assign awready = !write_addr_received_r;
    assign wready  = !write_data_received_r;
    always @(*) begin
        write_en = write_addr_received_r && write_data_received_r;
    end
    always @(*) begin
        write_addr_received_w = write_addr_received_r;
        if(awvalid && awready)
            write_addr_received_w = 1;
        else if(write_en)
            write_addr_received_w = 0;
    end
    always @(*) begin
        write_data_received_w = write_data_received_r;
        if(wvalid && wready)
            write_data_received_w = 1;
        else if(write_en)
            write_data_received_w = 0;
    end
    always @(*) begin
        write_addr_w = write_addr_r;
        if(awvalid && awready)
            write_addr_w = awaddr;
    end
    always @(*) begin
        write_data_w = write_data_r;
        if(wvalid && wready)
            write_data_w = wdata;
    end
    always @(posedge axis_clk or negedge axis_rst_n ) begin
        if(!axis_rst_n) begin
            write_addr_r <= 0;
            write_data_r <= 0;
            write_addr_received_r <= 0;
            write_data_received_r <= 0;
        end
        else begin
            write_addr_r <= write_addr_w;
            write_data_r <= write_data_w;
            write_addr_received_r <= write_addr_received_w;
            write_data_received_r <= write_data_received_w;
        end
    end
    // ---------------------------------------------------------------------------
    // AXI4-Lite Read Transaction
    // read_check coef(tap_ram), data_length, config_r
    assign arready = !rvalid && !write_en;
    assign rvalid  = read_valid_r;
    assign rdata   = read_data_r;
    
    always @(*) begin
        read_valid_w = read_valid_r;
        if(rvalid && rready)
            read_valid_w = 0;
        else if(read_tap_ram_r)
            read_valid_w = 1;
    end

    always @(*) begin
        if(read_data_length_r)
            read_data_r = data_length;
        else if(read_config_r)
            read_data_r = config_r;
        else
            read_data_r = tap_ram_data_r;
    end

    always @(*) begin
        read_addr_received_w = read_addr_received_r;
        if(arvalid && arready)
            read_addr_received_w = 1;
        else
            read_addr_received_w = 0;
    end

    always @(*) begin
        read_addr_w = read_addr_r;
        if(arvalid && arready)
            read_addr_w = araddr;
    end

    always @(*) begin
        read_data_length_w = read_data_length_r;
        if(arvalid && arready && araddr == 12'h10)
            read_data_length_w = 1;
        else if(rready && rvalid)
            read_data_length_w = 0;
    end

    always @(*) begin
        read_config_w = read_config_r;
        if(arvalid && arready && araddr == 12'h00)
            read_config_w = 1;
        else if(rready && rvalid)
            read_config_w = 0;
    end

    always @(*) begin
        read_tap_ram_w = read_tap_ram_r;
        if(read_addr_received_r)
            read_tap_ram_w = 1;
        else
            read_tap_ram_w = 0;
    end

    always @(*) begin
        tap_ram_data_w = tap_ram_data_r;
        if(read_tap_ram_r)
            tap_ram_data_w = tap_Do;
    end
    always @(posedge axis_clk or negedge axis_rst_n ) begin
        if(!axis_rst_n) begin
            read_addr_r          <= 0;
            read_addr_received_r <= 0;
            read_data_length_r   <= 0;
            read_config_r        <= 0;
            read_tap_ram_r       <= 0;
            tap_ram_data_r       <= 0;
            read_valid_r         <= 0;
        end
        else begin
            read_addr_r          <= read_addr_w;
            read_addr_received_r <= read_addr_received_w;
            read_data_length_r   <= read_data_length_w;
            read_config_r        <= read_config_w;
            read_tap_ram_r       <= read_tap_ram_w;
            tap_ram_data_r       <= tap_ram_data_w;
            read_valid_r         <= read_valid_w;
        end
    end
    
    // ---------------------------------------------------------------------------
    // bram for tap RAM
    
    assign tap_WE = {4{write_en && write_addr_r != 12'h00 && write_addr_r != 12'h10}};
    assign tap_EN = 1;
    assign tap_Di = write_data_r;
    assign tap_A  = tap_A_w;

    always @(*) begin
        tap_A_w = tap_A_r;
        if(read_addr_received_r && (read_addr_r != 12'h00 && read_addr_r != 12'h10))
            tap_A_w = read_addr_r - 12'h20;
        else if(write_en)
            tap_A_w = write_addr_r - 12'h20;
        else
            tap_A_w = shift_counter;
    end
    always @(posedge axis_clk or negedge axis_rst_n) begin
        if(!axis_rst_n)
            tap_A_r <= 0;
        else
            tap_A_r <= tap_A_w;
    end
    // ---------------------------------------------------------------------------
    // axi address config and data_length
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            config_r <= 32'h0000_0004;
        end
        else begin
            if(write_en && write_addr_r == 12'h00) config_r <= write_data_r;
            else if(ss_tvalid && state == SET_UP) config_r <= config_r & 32'hFFFF_FFFE;
            else if(sm_tready && sm_tvalid && sm_tlast) config_r <= 32'h0000_0006;
            else if(rready && rvalid && read_config_r && config_r[1] == 1) config_r <= config_r & 32'hFFFF_FFFD;
            else config_r <= config_r;
        end
    end
    
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_length <= 0;
        end
        else begin
            if(write_en && write_addr_r == 12'h10) data_length <= write_data_r;
            else data_length <= data_length;
        end
    end
    // ---------------------------------------------------------------------------
    // compute


    assign data_EN = (state == COMPUTE);
    assign data_write_en = (data_read_stage == 0);
    assign data_WE = {4{data_write_en}};
    assign data_A = (data_read_stage == 2) ? shift_counter - 4 : shift_counter;
    assign data_Di = mult1;

    assign mult2 = tap_Do;
    assign mult_result = mult1 * mult2;
    assign sm_tdata = accumulated_result;

    always@(*)begin
        if(shift_counter == 0)begin
            if(data_counter >= data_length) mult1 = 0;
            else mult1 = data_received;
        end
        else begin
            if(first_data) mult1 = 0;
            else mult1 = current_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_counter <= 0;
        end
        else begin
            if(sm_tready && sm_tvalid) data_counter <= data_counter + 1;
            else if(state_next == IDLE) data_counter <= 0;
            else data_counter <= data_counter;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_received <= 0;
        end
        else begin
            if(ss_tready && ss_tvalid) data_received <= ss_tdata;
            else data_received <= data_received;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            last_data <= 0;
        end
        else begin
            if(state_next == IDLE) last_data <= 0;
            else if(ss_tready && ss_tvalid) last_data <= ss_tlast;
            else last_data <= last_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            computing <= 0;
        end
        else begin
            if(ss_tready && ss_tvalid) computing <= 1;
            else if(shift_counter == 0 && data_write_en) computing <= 0;
            // else if(sm_tready && sm_tvalid && (data_counter >= data_length - 1) && !sm_tlast) computing <= 1;
            else computing <= computing;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            outputing <= 0;
        end
        else begin
            if(shift_counter == 0 && data_write_en) outputing <= 1;
            else if(sm_tready && sm_tvalid) outputing <= 0;
            else outputing <= outputing;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            data_read_stage <= 2;
        end
        else begin
            if(state == COMPUTE)begin
                if(sm_tready && sm_tvalid) data_read_stage <= 2;
                else if(computing) begin
                    if(data_read_stage == 0) data_read_stage <= 2;
                    else data_read_stage <= data_read_stage - 1;
                end
                else data_read_stage <= data_read_stage;
            end
            else begin
                data_read_stage <= data_read_stage;
            end
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            current_data <= 0;
        end
        else begin
            if(data_read_stage == 1) current_data <= data_Do;
            else current_data <= current_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            first_data <= 0;
        end
        else begin
            if(state == SET_UP && state_next == COMPUTE) first_data <= 1;
            else if(sm_tready && sm_tvalid) first_data <= 0;
            else first_data <= first_data;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            shift_counter <= 40;
        end
        else begin
            if(shift_counter == 0 && data_write_en) shift_counter <= 40;
            else if(data_write_en && shift_counter != 0) shift_counter <= shift_counter - 4;
            else shift_counter <= shift_counter;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            accumulated_result <= 0;
        end
        else begin
            if(sm_tready && sm_tvalid) accumulated_result <= 0;
            else if(data_read_stage == 0) accumulated_result <= accumulated_result + mult_result;
            else accumulated_result <= accumulated_result;
        end
    end

     // ss control
    always@(*)begin
        ss_tready = (!computing && !outputing && state == COMPUTE);
    end

    // sm control
    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            sm_tvalid <= 0;
        end
        else begin
            if(sm_tready && sm_tvalid) sm_tvalid <= 0;
            else if(shift_counter == 0 && data_write_en) sm_tvalid <= 1;
            else sm_tvalid <= sm_tvalid;
        end
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
        if(!axis_rst_n)begin
            sm_tlast <= 0;
        end
        else begin
            if(sm_tvalid && sm_tready) sm_tlast <= 0;
            else if(shift_counter == 0 && data_write_en) sm_tlast <= (data_counter == data_length -1);
            else sm_tlast <= sm_tlast;
        end
    end

endmodule