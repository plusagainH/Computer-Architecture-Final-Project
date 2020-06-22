module multDiv(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid, mode; // mode: 0: multu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 2'b00;
    parameter MULT = 2'b01;
    parameter DIV  = 2'b10;
    parameter OUT  = 2'b11;

    // Todo: Wire and reg
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;
    //self defined
    reg ending;

    // Todo 5: wire assignments
    assign out = shreg;
    assign ready = ending;

    // Combinational always block
    // Todo 1: State machine
    always @(negedge clk) begin
        case(state)
            IDLE: begin
                if (valid) begin
                    ending = 0;
                    if (mode) state_nxt = DIV;
                    else      state_nxt = MULT;
                end
                else state_nxt = IDLE;       
            end
            MULT: begin
                if (counter == 5'b11111) begin
                    state_nxt = OUT;
                    ending = 1;
                end
                else                    state_nxt = MULT;
            end
            DIV : begin
                if (counter == 5'b11111) begin
                    state_nxt = OUT;
                    ending = 1;
                end
                else                    state_nxt = DIV;
            end
            OUT : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(negedge clk) begin
        if (((state == MULT) || (state == DIV)) && (counter != 5'b11111))  counter_nxt = counter + 5'b00001;
        else begin
            counter_nxt = 0;
        end
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(negedge clk) begin
        if (state == MULT) begin
            alu_out = alu_in + shreg[63:32];
        end
        if (state == DIV) begin
            if (shreg[63:32] >= alu_in) begin
                alu_out[31:0] = shreg[63:32] - alu_in;
                alu_out[32] = 0;
            end
            else begin
                alu_out[31:0] = alu_in - shreg[63:32];
                alu_out[32] = 1;
            end
        end
    end
    
    // Todo 4: Shift register
    always @(negedge clk) begin
        if ((state == IDLE) && (valid == 1)) begin
            shreg_nxt[63:32] = 32'b00000000000000000000000000000000;
            shreg_nxt[31:0] = in_A;
            if (mode == 1) shreg_nxt = shreg_nxt << 1;
        end
        if (state == MULT) begin
            if (shreg[0] == 1) begin
                shreg_nxt = shreg >> 1;
                shreg_nxt[63:31] = alu_out[32:0];
            end
            else
                shreg_nxt = shreg >> 1;
        end
        if (state == DIV) begin
            if (alu_out[32] == 0) begin
                shreg_nxt[63:32] = alu_out[31:0];
                shreg_nxt = shreg_nxt << 1;
                shreg_nxt[0] = 1;
            end  
            else 
                shreg_nxt = shreg << 1;
            if (counter == 31) begin
                shreg_nxt[62:32] = shreg_nxt[63:33];
                shreg_nxt[63] = 0;
            end
        end
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= state_nxt;
            counter <= counter_nxt;
            shreg <= shreg_nxt;
            alu_in <= alu_in_nxt;
        end
    end

endmodule