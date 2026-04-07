/*
# Team ID:          1109
# Theme:            Maze Solver Bot
# Author List:      Vignesh M, Aashish M, Sree Balaji, Vidula Balamurugan
# Filename:         adc_controller.v
# File Description: SPI-based ADC controller module that reads 12-bit data 
#                   from external ADC (Channel 0) using serial clock and 
#                   chip select control.
# Global variables: None
*/

module adc_controller(
    input dout, adc_sck,
    output adc_cs_n, din, 
    output reg [11:0] d_out_ch0
);

/*
one read write cycle of adc is of 16 bits so we have named them 0, 2...15
out of that we have to update the address in MSB first format on 2, 3, 4 falling edges
we have to read from 4th to 15th rising edges
*/

    reg [3:0] din_counter = 0; // mod-15
    reg [3:0] sp_counter = 0;
    reg adc_cs = 1;
    reg din_temp = 0;       // default 0 since we are reading from adc channel 0
    reg [11:0] dout_chx = 0;
    reg adc_clk_reg = 0;
	 
    // data writing on negedge.
    always @(negedge adc_sck) begin
        din_counter <= din_counter + 1;
        // chip select
        if(din_counter == 0) begin
            adc_cs <= !adc_cs;
        end
    end

    always @(posedge adc_sck) begin
        // read the adc value in between 4th sclk cycle and 15th sclk cycle.
        if((sp_counter >= 4) && (sp_counter <= 15)) begin
            dout_chx[15 - sp_counter] <= dout; // fill in the data
        end else begin
            dout_chx <= 0; // reset the dout_chx
        end
        sp_counter <= sp_counter + 1'b1;
    end

    always @(posedge adc_sck ) begin
        if ((sp_counter == 15)&& (!adc_cs)) begin
            d_out_ch0 <= dout_chx;
        end
    end

    // output
    assign adc_cs_n = adc_cs;
    assign din = din_temp;

endmodule
