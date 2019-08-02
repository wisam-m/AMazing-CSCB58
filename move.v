// Part 2 skeleton

module move
	(
		CLOCK_50,
		KEY,
		SW,
		VGA_CLK,                           
        VGA_HS,                            
        VGA_VS,                            
        VGA_BLANK_N,                        
        VGA_SYNC_N,                        
        VGA_R,                           
        VGA_G,                            
        VGA_B,                         
        HEX0,
        HEX1,
		HEX2,
		HEX3,
		HEX4,
		LEDR,
		GPIO
	);
	
	input			CLOCK_50;           
   input   [9:0]   SW;
   input   [3:0]   KEY;
	input 	[40:0] 	GPIO;
   output  [6:0] 	HEX0;
   output  [6:0] 	HEX1;
	 
   output  [6:0] 	HEX2;
   output  [6:0] 	HEX3;
	output  [6:0]	HEX4;
	output          VGA_CLK;           
   output          VGA_HS;                  
   output          VGA_VS;                  
   output          VGA_BLANK_N;              
   output          VGA_SYNC_N;               
   output  [9:0] 	VGA_R;                  
   output  [9:0] 	VGA_G;                    
   output  [9:0] 	VGA_B;                   
	output	[17:0]	LEDR;
		
	wire			resetn;
	wire			load_x;
	wire			load_y;
	wire	[4:0]	state;
	wire	[6:0]	block_x;
	wire	[6:0]	block_y;
	wire	[2:0]	block_colour;
	wire			Up_block;
	wire			Down_block;
	wire			Left_block;
	wire			Right_block;
	wire	[1:0]	COL;
	wire	[6:0]	newscore;
	wire			reset_game;
	wire			plot;
	
	reg 	[2:0] 	colour;
	reg 	[6:0] 	x;
   reg 	[6:0] 	y;
	reg  			writeEn;
	reg 			speed = 2'b00;
	reg		[25:0]	block_counter = 26'b0;
	reg		[2:0]	curr_colour	= 3'b101; 	//purple
	reg		[6:0]	init_x = 7'b0001000;	//init_x = 8
	reg		[6:0]	init_y = 7'b0001000;	//init_y = 8
	
	
	assign resetn = 1'b1;
	assign LEDR[0] = GPIO[0]; // right = 1
	assign LEDR[1] = ~GPIO[1]; // left = 1
	assign LEDR[2] = GPIO[3]; // up = 1
	assign LEDR[3] = ~GPIO[2]; // down = 1
//	assign LEDR[0] = ~KEY[0]; // right = 1
//	assign LEDR[1] = ~KEY[3]; // left = 1
//	assign LEDR[2] = ~KEY[2]; // up = 1
//	assign LEDR[3] = ~KEY[1]; // down = 1
	assign LEDR[17] = Up_block;
	assign LEDR[16] = Down_block;
	assign LEDR[15] = Left_block;
	assign LEDR[14] = Right_block;
	
	vga_adapter VGA(
            .resetn(resetn),
            .clock(CLOCK_50),
            .colour(colour),
            .x(x),
            .y(y),
            .plot(writeEn),
            .VGA_R(VGA_R),
            .VGA_G(VGA_G),
            .VGA_B(VGA_B),
            .VGA_HS(VGA_HS),
            .VGA_VS(VGA_VS),
            .VGA_BLANK(VGA_BLANK_N),
            .VGA_SYNC(VGA_SYNC_N),
            .VGA_CLK(VGA_CLK));
   defparam VGA.RESOLUTION = "160x120";
   defparam VGA.MONOCHROME = "FALSE";
   defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
   defparam VGA.BACKGROUND_IMAGE = "LEVEL2.mif"; // can change to LEVEL0, LEVEL1, LEVEL2, LEVEL3
	
	
	
   datapath d0(
			.clk(CLOCK_50), 
			.load_x(load_x), 
			.load_y(load_y), 
			.init_x(init_x), 
			.init_y(init_y), 
			.resetn(resetn), 
			.x(block_x), 
			.y(block_y), 
			.colour(block_colour), 
			.plot(plot), 
			.state(state), 
			.Block_colour(curr_colour),
			.Up_block(Up_block),
			.Down_block(Down_block),
			.Left_block(Left_block),
			.Right_block(Right_block)
	);
	
	control c0(
			.clk(CLOCK_50), 
			.Right(GPIO[0]), 
			.Left(~GPIO[1]), 
			.Down(GPIO[3]), 
			.Up(~GPIO[2]), 
//			.Right(~KEY[0]), 
//			.Left(~KEY[3]), 
//			.Down(~KEY[1]), 
//			.Up(~KEY[2]), 
			.resetn(resetn), 
			.load_x(load_x), 
			.load_y(load_y), 
			.state(state), 
			.reset_game(reset_game), 
			.block_counter(block_counter), 
			.speed(speed));
	
	
	collision3 CC
	(
		.x(x),
		.y(y),
		.out(COL)
	);
	
	hex_display h0 (
		.hex_digit(y[3:0]), 
		.segments(HEX0)
	);
	
	hex_display h1 (
		.hex_digit(y[6:4]), 
		.segments(HEX1)
	);
	
	hex_display h2 (
		.hex_digit(x[3:0]), 
		.segments(HEX2)
	);
	
	hex_display h3 (
		.hex_digit(x[6:4]), 
		.segments(HEX3)
	);
	
	hex_display h4 (
		.hex_digit(COL), 
		.segments(HEX4)
	);
	
	always @(posedge CLOCK_50)
    begin
        if(plot) 
            begin
                writeEn <= plot;   
					 x <= block_x;
                y <= block_y;
                colour = block_colour;
            end
    end
       
endmodule

module control(
		clk, 
		Right, 
		Left, 
		Down, 
		Up, 
		resetn, 
		load_x, 
		load_y, 
		state, 
		reset_game, 
		block_counter, 
		speed
	);
   input 	[25:0] 	block_counter;
   input 			reset_game;
   input 			clk;
	input			Right;
	input			Left;
	input			Down;
	input			Up;
	input 			resetn;
   input 	[1:0] 	speed;
    
	output 	reg 		load_x;
	output 	reg			load_y;
	output 	reg [4:0] 	state;
    
	wire 	[26:0] 	move;   
   wire 				move_result;
	wire	[25:0] 	speed1= 26'b010001001010101000100; 
	wire	[25:0] 	speed2= 26'b011110100001001000000; 
	
	reg 	[3:0] 	current_state, next_state;
   reg 	[25:0] 	curr_speed;
	
    
   localparam    	Clear    	= 4'b0000;
   localparam 		LoadX    	= 4'b0001;
   localparam 		Wait   	 	= 4'b0010;
   localparam 		LoadY    	= 4'b0011;
   localparam    	Take    		= 4'b0100;
   localparam    	ClearAll   	= 4'b0101;
   localparam   	PlotR    	= 4'b0110;
   localparam    	PlotL    	= 4'b0111;
   localparam    	PlotD    	= 4'b1000;
   localparam    	PlotU    	= 4'b1001;
   localparam 		Curr_State 	= 4'b1010;
   localparam 		Plot			= 4'b1011;
   localparam 		Clean 		= 4'b1100;
	
	
    
    
    
   always @(*) begin
       if (speed == 2'b00) begin
          curr_speed <= speed1 + speed2;
	end
	end
	
   RateDivider(
		.clk(clk),
		.move(move),
		.resetn(resetn),
		.speed(curr_speed)
	);
     
   assign move_result = (move == block_counter) ? 1 : 0;
   
   always @(*)
   begin: state_table
        case (current_state)
            Clear: next_state = LoadX ;
            LoadX: next_state = Wait;
            Wait: next_state = LoadY;
            LoadY: next_state = Curr_State; 
				PlotR: next_state = reset_game ? LoadY : Plot;
            PlotL: next_state = reset_game ? LoadY : Plot;
            PlotD: next_state = reset_game ? LoadY : Plot;
            PlotU: next_state = reset_game ? LoadY : Plot;
            Plot: next_state= Curr_State;
            Curr_State: next_state = reset_game ? Clean : (((Right || Left || Down || Up) && move_result) ? ClearAll : LoadY );
            Clean: next_state = Clear;
			ClearAll: 
			begin
                if(Right)
                    next_state <= PlotR;
                else if (Left)
                    next_state <= PlotL;
                else if (Down)
                    next_state <= PlotD;
                else if (Up)
                    next_state <= PlotU;
            end      
        default: next_state = Clear;
        endcase
   end

   always@(*)
   begin: enable_signals
        load_x = 1'b0;
        load_y = 1'b0;
        state = 5'b00000;
        case (current_state)
            LoadX: begin
                load_x = 1'b1;
                end
            LoadY: begin
                load_y = 1'b1;
                end
            Clean: begin 
                state = 5'b00011;
                load_y = 1'b0;
                end
            ClearAll: begin
                state = 5'b00011;
                load_y = 1'b0;
                end
        
            PlotR: begin
                state = 5'b01000;
                load_y = 1'b0;
                end
           
            PlotD: begin
                state = 5'b00010;
                load_y = 1'b0;
                end
               
            PlotL: begin
                state = 5'b00100;
                load_y = 1'b0;
                end
               
            PlotU: begin
                state = 5'b00001;
                load_y = 1'b0;
                end
               
            Plot: begin
                state = 5'b10000;
                end
        endcase
   end

   always @(posedge clk)
   begin: states
        if(!resetn)
            current_state <= LoadX;
        else
            current_state <= next_state;
   end

endmodule


module datapath
	(
		clk,
		load_x,
		load_y,
		init_x,
		init_y,
		resetn,
		x,
		y,
		colour,
		plot,
		state,
		Block_colour,
		Up_block,
		Down_block,
		Left_block,
		Right_block
	);
	
	input 					clk;
	input 					load_x;
	input 					load_y;
	input 	[6:0] 		init_x;
	input 	[7:0] 		init_y;
	input 					resetn;
	output 	reg [6:0] 	x;
	output 	reg [6:0] 	y;
	output	reg [2:0]	colour;
	output	reg			plot;
	input		[3:0]			state;
	input 	[2:0] 		Block_colour;
	output 	[2:0]			Up_block;
	output	[2:0]			Down_block;
	output	[2:0]			Left_block;
	output	[2:0]			Right_block;
	wire		[6:0]			Up_y;
	wire		[6:0]			Down_y;
	wire		[6:0]			Left_x;
	wire		[6:0]			Right_x;
	wire		[6:0]			Norm_x;
	wire 		[6:0]			Norm_y;
	assign Up_y 	= y - 7'b0000010;
	assign Down_y 	= y + 7'b0000010;
	assign Left_x 	= x - 7'b0000010;
	assign Right_x = x + 7'b0000010;
	assign Norm_x 	= x;
	assign Norm_y 	= y;
	collision3 UB
	(
		.x(Norm_x),
		.y(Up_y),
		.out(Up_block)
	);
	
	collision3 DB
	(
		.x(Norm_x),
		.y(Down_y),
		.out(Down_block)
	);
	
	collision3 LB
	(
		.x(Left_x),
		.y(Norm_y),
		.out(Left_block)
	);
	
	collision3 RB
	(
		.x(Right_x),
		.y(Norm_y),
		.out(Right_block)
	);
	
	always @(posedge clk) begin
		if(!resetn) begin
			x <= 6'b0;
			x <= 6'b0;
			colour <= 3'b0;
		end
		else begin
			if (load_x) begin
				x[6:0] <= init_x;
				y[6:0] <= init_y;
				plot <= 1'b0;
			end
			else if (load_y) begin
				plot <= 1'b0;
			end
			//CLEAN state
			else if (state == 5'b00011) begin
				plot <= 1'b1;
				colour <= 3'b011;
			end
			//move_R state not block
			else if ((state == 5'b01000) && (Right_block == 2'b00)) begin
				plot <= 1'b1;
				x[6:0] <= x + 7'b0000010;
				colour <= Block_colour;
			end
			//move_R state block
			else if ((state == 5'b01000) && (Right_block == 2'b01)) begin
				plot <= 1'b1;
				colour <= Block_colour;
			end
			else if((state == 5'b01000) && (Right_block == 2'b10)) begin
				plot <= 1'b1;
				colour <= 3'b101;
				x[6:0] <= 7'b1111110;
				y[6:0] <= 7'b0100010;
			end
			//move_L state not block
			else if ((state == 5'b00100) && (Left_block == 2'b00)) begin
				plot <= 1'b1;
				x[6:0] <= x - 7'b0000010;
				colour <= Block_colour;
			end
			//move_L state block
			else if ((state == 5'b00100) && (Left_block == 2'b01)) begin
				plot <= 1'b1;
				colour <= Block_colour;
			end
			else if((state == 5'b00100) && (Left_block == 2'b10)) begin
				plot <= 1'b1;
				colour <= 3'b101;
				x[6:0] <= 7'b1111110;
				y[6:0] <= 7'b0100010;
			end
			//move_D state not block
			else if ((state == 5'b00010) && (Down_block == 2'b00)) begin
				plot <= 1'b1;
				y[6:0] <= y + 7'b0000010;
				colour <= Block_colour;
			end
			//move_D state block
			else if ((state == 5'b00010) && (Down_block == 2'b01)) begin
				plot <= 1'b1;
				colour <= Block_colour;
			end
			else if((state == 5'b00010) && (Down_block == 2'b10)) begin
				plot <= 1'b1;
				colour <= 3'b101;
				x[6:0] <= 7'b1111110;
				y[6:0] <= 7'b0100010;
			end
			
			//move_U state  not block
			else if ((state == 5'b00001) && (Up_block == 2'b00)) begin
				plot <= 1'b1;
				y[6:0] <= y - 7'b0000010;
				colour <= Block_colour;
			end
			//move_U state block
			else if ((state == 5'b00001) && (Up_block == 2'b01)) begin
				plot <= 1'b1;
				colour <= Block_colour;
			end
			else if((state == 5'b00001) && (Up_block == 2'b10)) begin
				plot <= 1'b1;
				colour <= 3'b101;
				x[6:0] <= 7'b1111110;
				y[6:0] <= 7'b0100010;
			end
			//PLOT state
			else if (state == 5'b10000) begin
				plot <= 1'b0;
			end
		end
	end
	

endmodule
   
   
module RateDivider (clk, move, resetn, speed);
    input [0:0] clk;
    input [0:0] resetn;
     input [25:0] speed;
    output reg [26:0] move; 
    always@(posedge clk) begin
        if (move == speed) begin 
            move <= 0; 
		end
        else if (clk == 1'b1) begin
            move <= move + 1'b1; 
		end
    end
endmodule


module hex_display(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule

