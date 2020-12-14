`timescale 1ns / 1ps

//`define REFRESH_MEM_MODE (1)

module top(
    input clk_in1,
    input rst_,
    output VGA_HS,
    output VGA_VS,
    output reg [3:0] VGA_R,
    output reg [3:0] VGA_G,
    output reg [3:0] VGA_B,
    input jump_raw,
    input right_raw,
    input left_raw,
    input stop_raw,
    output reg jumping
    );
    
    wire enable_V_counter;
    wire [15:0] H_count_value;
    wire [15:0] V_count_value;
    reg reset;
    reg pre_reset;
    wire clk;
    wire locked;

    
    always @ (posedge clk)
    begin
        pre_reset <= ~rst_;
        reset <= pre_reset;
    end
    
    wire [6:0] jump_vert;
    reg jump_raw_p1;
    reg jump;
    
    reg right_raw_p1;
    reg right;
    
    reg left_raw_p1;
    reg left;
    
    reg stop_raw_p1;
    reg stop;
    
    always @(posedge clk) 
    //Avoid metastability for inputs
    begin 
        jump_raw_p1 <= jump_raw;
        jump <= jump_raw_p1;
        
         right_raw_p1 <= right_raw;
         right <= right_raw_p1;      
         
         left_raw_p1 <= left_raw;
         left <= left_raw_p1;
         
         stop_raw_p1 <= stop_raw;
         stop <= stop_raw_p1;   
    end
    
    wire jump_posedge_pulse;
    wire jump_negedge_pulse;
    assign jump_negedge_pulse = jump & ~jump_raw_p1;
    assign jump_posedge_pulse = ~jump & jump_raw_p1;
      
    wire right_posedge_pulse;
    assign right_posedge_pulse = ~right & right_raw_p1;
   
    wire left_posedge_pulse;
    assign left_posedge_pulse = ~left & left_raw_p1;
    
    wire stop_posedge_pulse;
    assign stop_posedge_pulse = ~stop & stop_raw_p1;
   
    reg jumpdone;
    reg sprite_idle;
    always @(posedge clk) 
    //Sprite State Control
    begin 
        if (reset)
        begin
            jumping <= 0;
            sprite_idle <= 1;
        end
        else
        begin
        
            if (jump_posedge_pulse)
            begin
                jumping <= 1; 
            end
            
            if (jumpdone)
            begin
                jumping <= 0; 
            end
            
            if (stop_posedge_pulse)
            begin
                sprite_idle <= 1;
            end
            
            if (left_posedge_pulse)
            begin
                sprite_idle <= 0;
            end
            
            if (right_posedge_pulse)
            begin
                sprite_idle <= 0;
            end
            
         end
    end
   
    `define ACTIVE_Hend (639)
    `define FRONT_PORCH_Hend (655)
    `define SYNC_PULSE_Hend (751)
    `define BACKPORCH_Hend (799)
    
    `define ACTIVE_Vend (479)
    `define FRONT_PORCH_Vend (489)
    `define SYNC_PULSE_Vend (491)
    `define BACKPORCH_Vend (520)
    
    horizontal_counter #(.BACKPORCH_Hend(`BACKPORCH_Hend)) vga_horiz(clk, reset, enable_V_counter, H_count_value);
    vertical_counter #(.BACKPORCH_Vend(`BACKPORCH_Vend)) vga_vert(clk, reset, enable_V_counter, V_count_value);
    
    
    //outputs
    assign VGA_HS = ~(H_count_value > `FRONT_PORCH_Hend && H_count_value <= `SYNC_PULSE_Hend) ? 1'b1:1'b0;
    assign VGA_VS = ~(V_count_value > `FRONT_PORCH_Vend && V_count_value <= `SYNC_PULSE_Vend) ? 1'b1:1'b0;
    
        //code for sprite animation, basically just animates a square around the screen
     reg [9:0] length;
     reg [8:0] height;
     reg [6:0] vx_mag; //x magnitude
     reg [6:0] vy_mag; //y magnitude

     reg signed [9:0]  sprite1_top; //y coord
     reg signed [10:0] sprite1_left; //x coord
     reg signed [9:0]  sprite2_top; //y coord
     reg signed [10:0] sprite2_left; //x coord
     reg signed [9:0]  sprite3_top; //y coord
     reg signed [10:0] sprite3_left; //x coord

     reg signed [7:0] sprite1_vx_dir; //actual x direction
     reg signed [7:0] sprite1_vy_dir; //actual y direction
     reg signed [7:0] sprite2_vx_dir; //actual x direction
     reg signed [7:0] sprite2_vy_dir; //actual y direction
     reg signed [7:0] sprite3_vx_dir; //actual x direction
     reg signed [7:0] sprite3_vy_dir; //actual y direction


     reg [5:0] color;
    reg [7:0] oneSixtyCounter; 
    `ifndef REFRESH_MEM_MODE
   
        
    always @ (posedge clk)
    //Drive Vy of sprite 3 if jumping
     begin
         if(reset)
         begin
            oneSixtyCounter <= 0; //This is only counting to 20 so can be smaller
            sprite3_vy_dir <= 0; 
            jumpdone <= 0;
         end
         else
         begin
            jumpdone <= 0;
            if (jumping)
            begin
                if (H_count_value == 0 && V_count_value == 0)
                begin
                    if(oneSixtyCounter < 20)
                    begin
                        oneSixtyCounter <= oneSixtyCounter + 1;
                        sprite3_vy_dir <= -4;
                    end
                    else if ((oneSixtyCounter >= 20) && (oneSixtyCounter < 40))
                    begin
                        if (oneSixtyCounter != 0)
                        begin
                            sprite3_vy_dir <= 4;
                            oneSixtyCounter <= oneSixtyCounter + 1;
                        end
                    end
                    else
                    begin
                        sprite3_vy_dir <= 0;
                        oneSixtyCounter <= 0;
                        jumpdone <= 1;
                    end
                    
                end
            end
            else
            begin
                sprite3_vy_dir <= 0;
            end
         end
     end
    
    
   // Sprite animation mode
   always @ (posedge clk)
   //Drive All Sprite's X velocities
     begin
         if(reset)
         begin
             length <= 40;
             height <= 40;
             vx_mag <= 0;
             vy_mag <= 0;
             sprite1_vx_dir <= vx_mag; 
             sprite1_vy_dir <= vy_mag; 
             sprite2_vx_dir <= vx_mag; 
             sprite2_vy_dir <= vy_mag; 
             sprite3_vx_dir <= vx_mag; 
             //sprite3_vy_dir <= vy_mag; //Driven elsewhere
         end
         else //if (H_count_value == 0 && V_count_value == 0)
         begin
            
             if (right_posedge_pulse)  //Right input
             begin
                 sprite1_vx_dir <= 2;
                 sprite2_vx_dir <= 2;
                 sprite3_vx_dir <= 2;
             end
             else if (left_posedge_pulse) //Left Input
             begin
                sprite1_vx_dir <= -2; 
                sprite2_vx_dir <= -2;
                sprite3_vx_dir <= -2;
             end
             else if (stop_posedge_pulse) //Idle Input
             begin
                sprite1_vx_dir <= 0;
                sprite2_vx_dir <= 0;
                sprite3_vx_dir <= 0;
             end
         end
     end

     always @ (posedge clk)
     begin
         if(reset)
         begin
         //Sprite Starting location

             sprite1_top <= 250;
             sprite1_left <= 300;
             
             sprite2_top <= 250;
             sprite2_left <= 300;
             
             sprite3_top <= 250;
             sprite3_left <= 300;
         end
         else
         begin
             if(H_count_value == 0 && V_count_value == 0)
             begin
                sprite1_left <= sprite1_left + sprite1_vx_dir;
                sprite2_left <= sprite2_left + sprite2_vx_dir;
                sprite3_left <= sprite3_left + sprite3_vx_dir;
                sprite3_top <= sprite3_top + sprite3_vy_dir;
             
             end
         end
     end
 
 reg sprite1_visible; 
 reg sprite2_visible; 
 reg sprite3_visible; 
 
 reg [7:0] SlowDownSwitching;
 
 always @(posedge clk)
 //Drive Sprite Display
 begin
     if (reset)
     begin //Show sprite 1
        sprite1_visible <= 1'b1;
        sprite2_visible <= 1'b0;
        sprite3_visible <= 1'b0;
        SlowDownSwitching <= 0;
     end
     else
     begin
        if (jumpdone) //Jump is over, go back to sprite 1
        begin
            sprite1_visible <= 1;
            sprite2_visible <= 0;
            sprite3_visible <= 0;///////
        end
        if (H_count_value == 0 && V_count_value == 0)
        begin
            if (SlowDownSwitching % 8 == 0) //switch sprite every 8 counts
            begin
                SlowDownSwitching <= SlowDownSwitching + 1;
                if (jumping) //only show sprite 3
                begin
                    sprite1_visible <= 0;
                    sprite2_visible <= 0;
                    sprite3_visible <= 1;
                end
                else if (sprite_idle) //Only show sprite 1
                begin
                    sprite1_visible <= 1;
                    sprite2_visible <= 0;
                    sprite3_visible <= 0; 
                end
                else
                begin
                    sprite3_visible <= 0;
                    sprite1_visible <= ~sprite1_visible;
                    sprite2_visible <= ~sprite2_visible;
                    
                end
            end
            else
            begin
                SlowDownSwitching <= SlowDownSwitching + 1;
            end
        end
    end
 end
 
 wire [3:0] sprite1_red;
 wire [3:0] sprite1_grn;
 wire [3:0] sprite1_blu;
 wire sprite1_vld;
 wire [3:0] sprite2_red;
 wire [3:0] sprite2_grn;
 wire [3:0] sprite2_blu;
 wire sprite2_vld;
 wire [3:0] sprite3_red;
 wire [3:0] sprite3_grn;
 wire [3:0] sprite3_blu;
 wire sprite3_vld;

 
 sprite1 sprite1(
    .CLK                   (clk          ),
    .RESET                 (reset        ),
    .SPRITE_ORIGIN_OFFSET_X(sprite1_left ),
    .SPRITE_ORIGIN_OFFSET_Y(sprite1_top  ),
    .VISIBLE               (sprite1_visible         ),
    .RASTER_X              (H_count_value),
    .RASTER_Y              (V_count_value),
    .RED                   (sprite1_red  ),
    .GRN                   (sprite1_grn  ),
    .BLU                   (sprite1_blu  ),
    .VALID                 (sprite1_vld  ) );
 sprite2 sprite2(
    .CLK                   (clk          ),
    .RESET                 (reset        ),
    .SPRITE_ORIGIN_OFFSET_X(sprite2_left ),
    .SPRITE_ORIGIN_OFFSET_Y(sprite2_top  ),
    .VISIBLE               (sprite2_visible         ),
    .RASTER_X              (H_count_value),
    .RASTER_Y              (V_count_value),
    .RED                   (sprite2_red  ),
    .GRN                   (sprite2_grn  ),
    .BLU                   (sprite2_blu  ),
    .VALID                 (sprite2_vld  ) );
    
 sprite3 sprite3(
    .CLK                   (clk          ),
    .RESET                 (reset        ),
    .SPRITE_ORIGIN_OFFSET_X(sprite3_left ),
    .SPRITE_ORIGIN_OFFSET_Y(sprite3_top  ),
    .VISIBLE               (sprite3_visible         ),
    .RASTER_X              (H_count_value),
    .RASTER_Y              (V_count_value),
    .RED                   (sprite3_red  ),
    .GRN                   (sprite3_grn  ),
    .BLU                   (sprite3_blu  ),
    .VALID                 (sprite3_vld  ) ); 

 always @ (*)
    if (sprite3_vld)
    begin
        VGA_R = sprite3_red;
        VGA_G = sprite3_grn;
        VGA_B = sprite3_blu;
    end
    else if (sprite2_vld)
    begin
        VGA_R = sprite2_red;
        VGA_G = sprite2_grn;
        VGA_B = sprite2_blu;
    end
    else if (sprite1_vld)
    begin
        VGA_R = sprite1_red;
        VGA_G = sprite1_grn;
        VGA_B = sprite1_blu;
    end
    else
    begin
        VGA_R = 0;
        VGA_G = 0;
        VGA_B = 0;
    end

`else
    
    // Refresh memory mode
    reg [4:0] present_state;
    reg [4:0] next_state;
    parameter idle = 5'b00001;
    parameter edge_a = 5'b00010;
    parameter edge_b = 5'b00100;
    parameter edge_c = 5'b01000;
    parameter done = 5'b10000;
    wire [18:0] write_addr;
    wire [18:0] read_addr;
    reg [7:0] d_in;
    wire [7:0] d_out;
    reg write_enable;
    reg [9:0] x0, x1, x2;
    reg [8:0] y0, y1, y2;
    reg [9:0] half_length;
    reg [9:0] mar_x;
    reg [8:0] mar_y;
    
    //define every color so you can mix intensities

    assign read_addr = {H_count_value[9:0], V_count_value[8:0]}; //addra
    assign write_addr = {mar_x, mar_y}; //addrb

    always @ (posedge clk)
    begin
        if(reset)
        begin
            
        end
    end
    
    always @ (posedge clk)
    begin
        if(reset)
        begin
            present_state   <= idle;
            
            mar_x           <= x0;
            mar_y           <= y0;
            
            write_enable    <= 0;
            
            VGA_R <= 0;
            VGA_G <= 0;
            VGA_B <= 0;

            x0 <= 10'd20;
            y0 <= 9'd460; //111001100

            half_length <= 10'd300;

            x1 <= x0 + (2*half_length);
            y1 <= y0;
            x2 <= x0 + (half_length);
            y2 <= y0 - (half_length);

            d_in <= 8'b11111111;
        end
        else
        begin
            case(present_state)
                idle:
                begin
                    present_state   <= edge_a;
                    write_enable    <= 1;

                    mar_x <= x0;
                    mar_y <= y0;
                    
                    VGA_R <= 0;
                    VGA_G <= 0;
                    VGA_B <= 0;
        
                    x1 <= x0 + (2*half_length);
                    y1 <= y0;
                    x2 <= x0 + (half_length);
                    y2 <= y0 - (half_length);
                end
                    
                edge_a:
                begin
                    mar_x <= mar_x + 1;
                    if(mar_x == (x1-1))
                        present_state <= edge_b;
                end
                edge_b:
                begin
                    mar_x <= mar_x - 1;
                    mar_y <= mar_y - 1;
                    if(mar_x == (x2+1))
                        present_state <= edge_c;
                    
                end
                edge_c:
                begin
                    mar_x <= mar_x - 1;
                    mar_y <= mar_y + 1;
                    if(mar_x == (x0+1))
                        present_state <= done;
                end
                done:
                begin
                    x0 <= x0 + 2; //moves right 4
                    y0 <= y0 - 2; //moves up 4
                    d_in <= d_in - 1;
                    if ( half_length > 20)
                    begin
                        half_length <= half_length - 4;
                        present_state <= idle;
                    end
                    else
                        write_enable <= 0;
                end
                default:
                    present_state <= idle;
            endcase
            
            VGA_R <= {d_out[5:4],{2{d_out[4]}}};
            VGA_G <= {d_out[3:2],{2{d_out[2]}}};
            VGA_B <= {d_out[1:0],{2{d_out[0]}}};
        
        end
    end

`endif
    
    clk_wiz_0 CLKWIZ0(.clk_out1(clk), .resetn(1'b1), .locked(locked), .clk_in1(clk_in1));

endmodule
