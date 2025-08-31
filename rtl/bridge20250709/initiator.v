`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/04/30 15:43:54
// Design Name: 
// Module Name: ireq
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
 

module initiator(
    //bram  
    input       [11:0]  bram_addr_a,
    input               bram_clk_a,
    input       [31:0]  bram_wrdata_a,
    output reg  [31:0]  bram_rddata_a,
    input               bram_en_a,
    input               bram_rst_a,
    input       [3:0]   bram_we_a,  
    
    //input
    input link_initialized,
    input port_initialized,
    input mode_1x,   

    input                                   m_axi_aclk,
    input                                   m_axi_aresetn,
    
    // w addr
    output  [4-1:0]                         m_axi_awid,
    output  [32-1:0]                        m_axi_awaddr,
    output                                  m_axi_awvalid,
    output  [7:0]                           m_axi_awlen,
    input                                   m_axi_awready,
    output  [1:0]                           m_axi_awburst,
    output     [2:0]                        m_axi_awsize,
    
    // w data
    output  [64-1:0]                        m_axi_wdata,
    output  [64/8-1:0]                      m_axi_wstrb,
    output   reg                            m_axi_wvalid,
    output   reg                            m_axi_wlast,
    input                                   m_axi_wready,
    
    // w response
    input [1:0]                             m_axi_bresp,
    input                                   m_axi_bvalid,
    output                                  m_axi_bready,
    
    // r addr
    output     [2:0]                        m_axi_arsize,
    output  [1:0]                           m_axi_arburst,
    output  [31:0]                          m_axi_araddr,
    output  [4-1:0]                         m_axi_arid,
    output  reg                             m_axi_arvalid,
    output  [7:0]                           m_axi_arlen,
    input                                   m_axi_arready,
    
    //rd data
    input [63:0]                            m_axi_rdata,
    input [1:0]                             m_axi_rresp,
    input                                   m_axi_rvalid,
    input                                   m_axi_rlast,
    output   reg                            m_axi_rready,


    input             log_clk,
    input             log_rstn,

    //resquest to  srio_gen2
    output reg        val_ireq_tvalid,
    input             val_ireq_tready,
    output reg        val_ireq_tlast,
    output reg [63:0] val_ireq_tdata,
    output      [7:0] val_ireq_tkeep,
    output     [31:0] val_ireq_tuser,

    //response from srio_gen2
    input             val_iresp_tvalid,
    output            val_iresp_tready,
    input             val_iresp_tlast,
    input      [63:0] val_iresp_tdata,
    input       [7:0] val_iresp_tkeep,
    input      [31:0] val_iresp_tuser,

	// to target
	input			  	irq_target,
	output			  	irq_srio,
    output 	[31:0] 		waddr,
    output 	[31:0] 		waddr_mask,
    output 	[31:0] 		raddr,
    output 	[31:0] 		raddr_mask,
	input	[31:0]		target_doorbell_info,
	output	[1:0]		srio_ip_rst,
	output				bit_inverse,
    
    input [223:0]                           phy_debug
    );
    localparam [3:0] NREAD  = 4'h2;
    localparam [3:0] NWRITE = 4'h5;
    localparam [3:0] SWRITE = 4'h6;
    localparam [3:0] DOORB  = 4'hA;
    localparam [3:0] MESSG  = 4'hB;
    localparam [3:0] RESP   = 4'hD;
    localparam [3:0] FTYPE9 = 4'h9;
    
    localparam [3:0] TNWR   = 4'h4;
    localparam [3:0] TNWR_R = 4'h5;
    localparam [3:0] TNRD   = 4'h4;
    
    localparam [3:0] TNDATA = 4'h0;
    localparam [3:0] MSGRSP = 4'h1;
    localparam [3:0] TWDATA = 4'h8;
    localparam LEN = 31;
    
	reg [7:0] m_srio_req_type;//0
    reg [LEN:0] m_srio_req_length;//1
    reg [31:0] m_srio_req_src_addr;//2
    reg [31:0] m_srio_req_dst_addr;//3
    reg [15:0] m_srio_req_src_id;//4
    reg [15:0] m_srio_req_dst_id;
    reg m_srio_req_en;//5
	reg m_srio_req_en_r;
	reg m_srio_req_en_rr;
    reg [7:0] byte_lane_0,byte_lane_1,byte_lane_2,byte_lane_3,byte_lane_4,byte_lane_5,byte_lane_6;//7
	
	
	reg ireq_done;
	reg iresp_done;

	reg [7:0] m_axi_req_len;
	reg [LEN:0] m_axi_nxt_rem_length;
	
	reg [223:0] phy_debug_r;
	reg [31:0] PNAs_cnt,SPNA_cnt,PNAd_cnt,Oerr_cnt,Ierr_cnt;
	reg [63:0] timer;

	reg 	irq_target_indx;
	reg 	irq_iresp_done_indx	;
	wire    irq_en_target		;
	wire    irq_clr_target		;
	wire    irq_en				;
	wire    irq_clr				;
	reg	[1:0]	srio_ip_rst 	;
	reg 	[31:0] doorbell_info;
	wire    bit_inverse			;

    always @(posedge log_clk)  begin
        if(!log_rstn) 
            bram_rddata_a <= 0;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==0)
            bram_rddata_a <= {24'b0,m_srio_req_type};
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==1)
            bram_rddata_a <= m_srio_req_length; //bytes
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==2)
            bram_rddata_a <= m_srio_req_src_addr;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==3)
            bram_rddata_a <= m_srio_req_dst_addr;    
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==4)
            bram_rddata_a <= {m_srio_req_src_id,m_srio_req_dst_id};
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==5)
            bram_rddata_a <= {31'b0,m_srio_req_en};
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==6)
            bram_rddata_a <= {27'b0,iresp_done,ireq_done,mode_1x,link_initialized,port_initialized};  
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==7)
            bram_rddata_a <= {byte_lane_3,byte_lane_2,byte_lane_1,byte_lane_0}; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==8)
            bram_rddata_a <= {4'b0,byte_lane_6,byte_lane_5,byte_lane_4}; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==9)
            bram_rddata_a <= PNAs_cnt; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==10)
            bram_rddata_a <= SPNA_cnt; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==11)
            bram_rddata_a <= PNAd_cnt; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==12)
            bram_rddata_a <= Oerr_cnt;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==13)
            bram_rddata_a <= Ierr_cnt;   
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==14)
            bram_rddata_a <= timer[63:32];  
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==15)
            bram_rddata_a <= timer[31:0]; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==16)
            bram_rddata_a <= {14'b0, irq_target_indx, irq_iresp_done_indx}; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==17)
            bram_rddata_a <= {12'b0,irq_clr_target,irq_clr, irq_en_target,irq_en}; 
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==18)
            bram_rddata_a <= waddr;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==19)
            bram_rddata_a <= waddr_mask;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==20)
            bram_rddata_a <= raddr;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==21)
            bram_rddata_a <= raddr_mask;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==22)
            bram_rddata_a <= doorbell_info;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==23)
            bram_rddata_a <= target_doorbell_info;
        else if(bram_en_a && bram_we_a==0 && bram_addr_a[11:2]==24)
            bram_rddata_a <= {30'b0,srio_ip_rst};
    end
	

	wire irq_srio = (irq_target_indx &&  irq_en_target) || (irq_iresp_done_indx && irq_en);
	
	reg irq_target_ff1;

    always @(posedge log_clk) begin
        if(!log_rstn) 
			irq_target_ff1 <= 1'b0;
		else begin
			irq_target_ff1 <= irq_target;
		end
	end

    always @(posedge log_clk) begin
        if(!log_rstn) 
			irq_target_indx <= 1'b0;
		else begin
			if((!irq_en_target) || (irq_target_indx && irq_clr_target))
				irq_target_indx <= 1'b0;
			else begin
				if((!irq_target_ff1) && irq_target) //posedge irq
					irq_target_indx <= 1'b1;
				else;
			end
		end
	end
	
	reg irq_iresp_done_ff1;
    always @(posedge log_clk) begin
        if(!log_rstn) 
			irq_iresp_done_ff1 <= 'b0;
		else begin
			irq_iresp_done_ff1 <= iresp_done; 
		end
	end

	reg irq_iresp_done_indx;
    always @(posedge log_clk) begin
        if(!log_rstn) 
			irq_iresp_done_indx <= 1'b0;
		else begin
			if((!irq_en) || (irq_iresp_done_indx && irq_clr))
				irq_iresp_done_indx <= 1'b0;
			else begin
				if((!irq_iresp_done_ff1) && iresp_done) //posedge irq_iresp_done_indx
					irq_iresp_done_indx <= 1'b1;
				else;
			end
		end
	end

    //m_srio_req_length
    reg [31:0] m_srio_req_length;
    always @(posedge log_clk) begin
        if(!log_rstn) 
            m_srio_req_length <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==1)
            m_srio_req_length <= bram_wrdata_a;
		else;
    end       
    
    //m_srio_req_src_addr
    always @(posedge log_clk) begin
        if(!log_rstn) 
            m_srio_req_src_addr <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==2)
            m_srio_req_src_addr <= bram_wrdata_a;   
		else;
    end 
   
    //m_srio_req_dst_addr
    always @(posedge log_clk) begin
        if(!log_rstn) 
            m_srio_req_dst_addr <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==3)
            m_srio_req_dst_addr <= bram_wrdata_a;       
		else;
    end
    
    //m_srio_req_id
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            m_srio_req_src_id <= 0;
            m_srio_req_dst_id <= 0;
        end
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==4) begin
            m_srio_req_src_id <= bram_wrdata_a[31:16]; 
            m_srio_req_dst_id <= bram_wrdata_a[15:0];
        end                      
		else;
    end
    //m_srio_req_dst_addr
    always @(posedge log_clk) begin
        if(!log_rstn) 
            m_srio_req_en <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==5)
            m_srio_req_en <= bram_wrdata_a;       
		else;
    end
    //m_srio_req_type
    always @(posedge log_clk) begin
        if(!log_rstn) 
            m_srio_req_type <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==0)
            m_srio_req_type <= bram_wrdata_a[7:0];         
		else;
    end
    //irq_en irq_clr
	//
    //        bram_rddata_a <= {12'b0, irq_en_target,irq_clr_target,irq_en,irq_clr}; 
	reg [31:0] irq_ctrl;
    always @(posedge log_clk) begin
        if(!log_rstn) 
            irq_ctrl <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==17)
            irq_ctrl <= bram_wrdata_a;       
		else;
    end

	assign irq_en_target 	= irq_ctrl[1];
	assign irq_en			= irq_ctrl[0];
	
	assign irq_clr_target  = irq_ctrl[3];
	assign irq_clr         = irq_ctrl[2];

	// local wirte base addr
	reg [31:0] waddr;
    always @(posedge log_clk) begin
        if(!log_rstn) 
            waddr <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==18)
            waddr <= bram_wrdata_a;       
		else;
    end
	// local write  base addr mask
	reg [31:0] waddr_mask;
    always @(posedge log_clk) begin
        if(!log_rstn) 
            waddr_mask <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==19)
            waddr_mask <= bram_wrdata_a;       
		else;
    end	
	
	// local read base addr
	reg [31:0] raddr;
    always @(posedge log_clk) begin
        if(!log_rstn) 
            raddr <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==20)
            raddr <= bram_wrdata_a;       
		else;
    end
	// local  read base addr mask
	reg [31:0] raddr_mask;
    always @(posedge log_clk) begin
        if(!log_rstn) 
            raddr_mask <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==21)
            raddr_mask <= bram_wrdata_a;       
		else;
    end

	// 
    always @(posedge log_clk) begin
        if(!log_rstn) 
            doorbell_info <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==22)
            doorbell_info <= bram_wrdata_a;       
		else;
    end
    always @(posedge log_clk) begin
        if(!log_rstn) 
            srio_ip_rst <= 'b10;   // bit 1 inverse
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==24)
            srio_ip_rst <= bram_wrdata_a;       
		else;
    end

	// local base addr
	//reg [31:0] target_doorbell_info;
	//always @(posedge log_clk) begin
	//    if(!log_rstn) 
	//        doorbell_info <= 0;
	//    else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==22)
	//        doorbell_info <= bram_wrdata_a;       
	//	else;
	//end
    //byte_lane
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            byte_lane_0 <= 0;
            byte_lane_1 <= 0;
            byte_lane_2 <= 0;
            byte_lane_3 <= 0;
            byte_lane_4 <= 0;
            byte_lane_5 <= 0;
            byte_lane_6 <= 0;
        end
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==7) begin
            byte_lane_0 <= bram_wrdata_a[7:0];    
            byte_lane_1 <= bram_wrdata_a[15:8]; 
            byte_lane_2 <= bram_wrdata_a[23:16]; 
            byte_lane_3 <= bram_wrdata_a[31:24];                       
        end
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==8) begin
            byte_lane_4 <= bram_wrdata_a[7:0];    
            byte_lane_5 <= bram_wrdata_a[15:8]; 
            byte_lane_6 <= bram_wrdata_a[23:16];                       
        end  
		else;
    end

    //phy_debug_r
    always @(posedge log_clk) begin
        if(!log_rstn) 
            phy_debug_r <= 0;
        else 
            phy_debug_r <= phy_debug; 
    end

    //PNAs_cnt
    always @(posedge log_clk) begin 
        if(!log_rstn) 
            PNAs_cnt <= 0;
        else if(phy_debug[98] && !phy_debug_r[98])    
            PNAs_cnt <= PNAs_cnt + 1;
		else;
    end
    
    //SPNA_cnt
    always @(posedge log_clk) begin
        if(!log_rstn) 
            SPNA_cnt <= 0;
        else if(phy_debug[160] && !phy_debug_r[160])    
            SPNA_cnt <= SPNA_cnt + 1;
		else;
    end
    
    //PNAd_cnt
    always @(posedge log_clk) begin
        if(!log_rstn) 
            PNAd_cnt <= 0;
        else if(phy_debug[161] && !phy_debug_r[161])    
            PNAd_cnt <= PNAd_cnt + 1;    
		else;
    end    
    
    //Oerr_cnt
    always @(posedge log_clk) begin
        if(!log_rstn) 
            Oerr_cnt <= 0;
        else if(phy_debug[164] && !phy_debug_r[164])    
            Oerr_cnt <= Oerr_cnt + 1;  
		else;
    end
    
    //Ierr_cnt        
    always @(posedge log_clk) begin
        if(!log_rstn) 
            Ierr_cnt <= 0;
        else if(phy_debug[166] && !phy_debug_r[166])    
            Ierr_cnt <= Ierr_cnt + 1;   
		else;
    end 
    
    //timer
    always @(posedge log_clk) begin
        if(!log_rstn) 
            timer <= 0;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==14) 
            timer[63:32] <= bram_wrdata_a;
        else if(bram_en_a && bram_we_a==4'hf && bram_addr_a[11:2]==15) 
            timer[31:0] <= bram_wrdata_a; 
        else
            timer <= timer + 1;   
    end     

    //  nwrite nwrite_r swrite doorbell
    //  r addr  channel
    //  output  [2:0]                        m_axi_arsize,
    //  output  [1:0]                           m_axi_arburst,
    //  output  [31:0]                          m_axi_araddr,
    //  output  [4-1:0]                         m_axi_arid,
    //  output                                  m_axi_arvalid,
    //  output  [7:0]                           m_axi_arlen,
    //  input                                   m_axi_arready,

    //wire [15:0] src_id;
    wire  [1:0] prio;
    wire  [7:0] tid;
    wire [63:0] header_beat;
    
    wire  [3:0] current_ftype;
    wire  [3:0] current_ttype;
    wire  [7:0] current_size;

    //m_srio_req_en_r
    always @(posedge log_clk)  begin
        if(!log_rstn)  begin
            m_srio_req_en_r <=1'b0;
            m_srio_req_en_rr <=1'b0;
		end
        else begin
            m_srio_req_en_r <= m_srio_req_en;
            m_srio_req_en_rr <= m_srio_req_en_r;
		end
    end

    //payload size for hello packet 
    //
    reg [7:0] pld_size;
	reg [31:0] pld_rem_size;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            pld_rem_size <= 0;
            pld_size <= 0;
        end
        else if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin 
            if(m_srio_req_en && (!m_srio_req_en_r)) begin
                if(m_srio_req_length[LEN:8]!=0) begin
                    pld_rem_size <= m_srio_req_length - 256;
                    pld_size <= 255;
                end
                else if(m_srio_req_length!=0) begin  
                    pld_rem_size <= 0;
                    pld_size <= (|m_srio_req_length[2:0])? m_srio_req_length[7:3]:(m_srio_req_length[7:3]-1); 
                end 
            end
            else if(val_ireq_tvalid && val_ireq_tready && val_ireq_tlast) begin  //next packet header
                if(pld_rem_size[LEN:8]!=0) begin
                    pld_rem_size <= pld_rem_size- 256;
                    pld_size <= 255;
                end
                else if(pld_rem_size!=0) begin  
                    pld_rem_size <= 0;
                    pld_size <= (|pld_rem_size[2:0])?pld_rem_size[7:3]:(pld_rem_size[7:3]-1);
                end
            end
        end 
        else if(m_srio_req_type == 8'h24) begin
            if(m_srio_req_en && (!m_srio_req_en_r)) begin
                if(m_srio_req_length[LEN:8]!=0) begin
                    pld_rem_size <= m_srio_req_length - 256;
                    pld_size <= 255;
                end
                else if(m_srio_req_length!=0) begin  
                    pld_rem_size <= 0;
                    pld_size <= (|m_srio_req_length[2:0])? m_srio_req_length[7:3]:(m_srio_req_length[7:3]-1); 
                end 
                else;
            end
            else begin
                if(val_ireq_tready && val_ireq_tvalid && val_ireq_tlast) begin
                    if(pld_rem_size[LEN:8]!=0) begin
                        pld_rem_size<= pld_rem_size - 256;
                        pld_size<= 255;
                    end
                    else if(pld_rem_size!=0) begin  
                        pld_rem_size<= 0;
                        pld_size<= (|pld_rem_size[2:0])?pld_rem_size[7:3]:(pld_rem_size[7:3]-1);
                    end
                end
                else;
            end

        end
    end


	reg [7:0] frm_cnt_r;
	wire [7:0] frm_cnt;

    always @(posedge log_clk)  begin
        if(!log_rstn) 
             frm_cnt_r <= 0;
	    else 
             frm_cnt_r <= frm_cnt;
	end

	assign frm_cnt = (val_ireq_tvalid && val_ireq_tready && val_ireq_tlast )? ( frm_cnt_r +1):( frm_cnt_r); 


    // dst addr ,farend
    reg [31:0] dst_addr_c;
    reg [31:0] dst_addr;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            dst_addr <= 0;
        else
        	dst_addr <= dst_addr_c;
    end

    always @(*)  begin
        if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin 
            if(m_srio_req_en && (!m_srio_req_en_r)) 
                dst_addr_c = m_srio_req_dst_addr;
            else if(val_ireq_tvalid && val_ireq_tready &&val_ireq_tlast) 
                dst_addr_c = dst_addr + ({20'b0,(pld_size+9'b1)});   
            else
                dst_addr_c = dst_addr;
        end 
        else
           	dst_addr_c = dst_addr;
    end


    //read src addr
    reg [31:0] src_addr;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            src_addr <= 0;
        else if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin 
            if(m_srio_req_en && (!m_srio_req_en_r)) 
                src_addr <= m_srio_req_src_addr;
            else if(m_axi_arvalid && m_axi_arready) 
                //src_addr <= src_addr + ({20'b0,(m_axi_req_len+9'b1)}<<3);   
                src_addr <= src_addr + ({20'b0,(m_axi_req_len+9'b1)});   
            else;
        end 
        else;
    end

     // far end addr for nread
    reg [31:0] farend_addr;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
             farend_addr <= 0;
        else if(m_srio_req_type == 8'h24) begin 
            if(m_srio_req_en && (!m_srio_req_en_r)) 
                farend_addr <= m_srio_req_dst_addr;
            else if(val_ireq_tready && (pld_rem_size !=0)) begin
                //farend_addr <= farend_addr + ({20'b0,(pld_size+9'b1)}<<3);   
                farend_addr <= farend_addr + ({20'b0,(pld_size+9'b1)});   
            end
            else;
        end 
        else;
    end
    assign m_axi_arsize = 3'h3; //64bit ,8byte 
    assign m_axi_arburst = 2'h1;
    assign m_axi_araddr = src_addr;
    assign m_axi_arid = 0; 
    assign m_axi_arlen = (((m_axi_req_len+1) >>3)-1);

    reg [31:0] m_axi_nxt_rem_length;
    //m_axi_arvalid
    always @(posedge log_clk) begin
        if(!log_rstn) 
            m_axi_arvalid <= 0;
        else begin
            if((m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) && m_srio_req_en && (!m_srio_req_en_r) && m_srio_req_length!=0)
                m_axi_arvalid <= 1;
            else if(m_axi_arvalid && m_axi_arready && m_axi_nxt_rem_length!=0)
                m_axi_arvalid <= 1;
            else if(m_axi_arvalid && m_axi_arready)
                m_axi_arvalid <= 0;    
            else;
        end
    end
    
    // -------------------       req channel    ---------------------
    // output reg        val_ireq_tvalid,
    // input             val_ireq_tready,
    // output reg        val_ireq_tlast,
    // output reg [63:0] val_ireq_tdata,
    // output      [7:0] val_ireq_tkeep,
    // output     [31:0] val_ireq_tuser,
    //
    //
     //rd data channel
    //input [63:0]                            m_axi_rdata,
    //input [1:0]                             m_axi_rresp,
    //input                                   m_axi_rvalid,
    //input                                   m_axi_rlast,
    //output                                  m_axi_rready,/

  wire        ireq_advance_condition  = val_ireq_tready && val_ireq_tvalid;
  wire        m_axi_r_advance_condition  = m_axi_rready && m_axi_rvalid;

  //val_ireq_tlast  
  reg [5:0] current_beat_cnt;
  always @(posedge log_clk) begin
    if (!log_rstn) begin
      current_beat_cnt <= 6'h0;
    end else if (ireq_advance_condition && val_ireq_tlast) begin
      current_beat_cnt <= 6'h0;
    end else if (ireq_advance_condition) begin
      current_beat_cnt <= current_beat_cnt + 1'b1;
    end
  end

    //seg_beat_cnt;
    reg seg_beat_cnt;
    reg seg_beat_cnt_r;
    always @(*) begin
        if (m_axi_rready &&  m_axi_rvalid && m_axi_rlast) begin
            seg_beat_cnt = 6'h0;
        end else if (m_axi_r_advance_condition) begin
            seg_beat_cnt = seg_beat_cnt_r + 1'b1;
        end 
        else
            seg_beat_cnt = seg_beat_cnt_r;
    end

    //seg_beat_cnt_r 
    always@(posedge log_clk) begin
        if (!log_rstn) begin
            seg_beat_cnt_r <= 6'h0;
        end 
        else begin
            seg_beat_cnt_r <= seg_beat_cnt;
        end
    end

    // total_beat_cnt;
    reg total_beat_cnt;
    reg total_beat_cnt_r;
    always @(*) begin
        if (m_srio_req_en && (!m_srio_req_en_r)) begin
            total_beat_cnt = 6'h0;
        end else if (m_axi_rready && m_axi_rvalid) begin
            total_beat_cnt = total_beat_cnt_r + 1'b1;
        end
        else
            total_beat_cnt = total_beat_cnt_r;
    end

    always@(posedge log_clk) begin
        if (!log_rstn) begin
            total_beat_cnt_r <= 6'h0;
        end 
        else begin
            total_beat_cnt_r <= total_beat_cnt;
        end

    end

    // req_len   rem_len
    //
    reg [7:0] m_axi_req_len;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            m_axi_nxt_rem_length <= 0;
            m_axi_req_len <= 0;
        end
        else if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin 
            if(m_srio_req_en && (!m_srio_req_en_r)) begin
                if(m_srio_req_length[LEN:8]!=0) begin
                    m_axi_nxt_rem_length <= m_srio_req_length - 256;
                    m_axi_req_len <= 255;
                end
                else if(m_srio_req_length!=0) begin  
                    m_axi_nxt_rem_length <= 0;
                    m_axi_req_len <= (|m_srio_req_length[2:0])? m_srio_req_length[7:3]:(m_srio_req_length[7:3]-1); 
                end 
            end
            else if(m_axi_arvalid && m_axi_arready) begin
                if(m_axi_nxt_rem_length[LEN:8]!=0) begin
                    m_axi_nxt_rem_length <= m_axi_nxt_rem_length - 256;
                    m_axi_req_len <= 255;
                end
                else if(m_axi_nxt_rem_length!=0) begin  
                    m_axi_nxt_rem_length <= 0;
                    m_axi_req_len <= (|m_axi_nxt_rem_length[2:0])? m_axi_nxt_rem_length[7:3]:(m_axi_nxt_rem_length[7:3]-1);
                end
            end
        end 
        else if(m_srio_req_type == 8'h24) begin
            if(m_srio_req_en && (!m_srio_req_en_r)) begin
                if(m_srio_req_length[LEN:8]!=0) begin
                    m_axi_nxt_rem_length <= m_srio_req_length - 256;
                    m_axi_req_len <= 255;
                end
                else if(m_srio_req_length!=0) begin  
                    m_axi_nxt_rem_length <= 0;
                    m_axi_req_len <= (|m_srio_req_length[2:0])? m_srio_req_length[7:3]:(m_srio_req_length[7:3]-1); 
                end 
                else;
            end
            else begin
                if(val_ireq_tready && val_ireq_tvalid) begin
                    if(m_axi_nxt_rem_length[LEN:8]!=0) begin
                        m_axi_nxt_rem_length <= m_axi_nxt_rem_length - 256;
                        m_axi_req_len <= 255;
                    end
                    else if(m_axi_nxt_rem_length!=0) begin  
                        m_axi_nxt_rem_length <= 0;
                        m_axi_req_len <= (|m_axi_nxt_rem_length[2:0])? m_axi_nxt_rem_length[7:3]:(m_axi_nxt_rem_length[7:3]-1);
                    end
                end
                else;
            end

        end
    end



	reg m_axi_rlast_r;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
			 m_axi_rlast_r <= 1'b0;
			 end
		else begin
			 m_axi_rlast_r <= m_axi_rlast;
		end
	 end


	reg m_axi_rready_r;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
	 		m_axi_rready_r <= 1'b0;
	 	end
		else begin
			m_axi_rready_r  <=m_axi_rready;
		end
	 end
	 
	reg m_axi_rvalid_r;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
	 		m_axi_rvalid_r <= 1'b0;
	 	end
		else begin
			m_axi_rvalid_r  <=m_axi_rvalid;
		end
	 end

    //m_axi_rready m_axi_rlast 
    always @(*) begin
        if(!link_initialized) begin
            m_axi_rready = 0;  
        end
        else if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin //nwrite swrite
            if (m_srio_req_en && (!m_srio_req_en_r))  begin
                m_axi_rready = 0;  
            end 
            else if(val_ireq_tready) begin
                if(( pld_rem_size !=0 ) &&(m_axi_rready_r && m_axi_rvalid_r && m_axi_rlast_r ))begin  //next packet
                    m_axi_rready = 0;
                end
                else if(val_ireq_tlast) begin //tlast hold
                    m_axi_rready = 0;
				end
                else begin
                    m_axi_rready =  val_ireq_tready;
                end
            end
            else begin
                m_axi_rready =  val_ireq_tready;
            end
        end
        else if((m_srio_req_type == 8'h24 )  || (m_srio_req_type == 8'ha0)) begin  //nread  && doorbell  no read data
            m_axi_rready = 0;
        end
    end


    wire [15:0] src_id = m_srio_req_src_id;
    wire [15:0] dst_id = m_srio_req_dst_id;

    wire [63:0] header_beat;

    assign   current_ftype = m_srio_req_type[7:4];
    assign   current_ttype = m_srio_req_type[3:0];
    assign   prio = 2'h1;

    wire  [31:0] srio_addr;   // 
    assign srio_addr = 	( m_srio_req_type == 8'h24) ?  	farend_addr :
						( m_srio_req_type == 8'ha0) ?  	{doorbell_info[15:0],16'h0}:
														dst_addr_c;
    //                       8         4              4            1     2     1         8     1     1      2    32
    assign header_beat  = {frm_cnt, current_ftype, current_ttype, 1'b0, prio, 1'b0, pld_size ,1'b1,1'b1,  2'b0, srio_addr};

	wire [63:0]  m_axi_rdata_sel;
	reg [63:0]  m_axi_rdata_inv;
	integer i;
	always @(*) begin
		for(i =0 ;i <8; i=i+1) begin
			  m_axi_rdata_inv[(i+1)*8-1 -:8] =  m_axi_rdata[(8-i)*8-1 -:8];
		end
	end
	assign   m_axi_rdata_sel =  bit_inverse ? m_axi_rdata_inv : m_axi_rdata;
    //val_ireq_tdata  
    always @(posedge log_clk) begin
        if (!log_rstn) begin 
            val_ireq_tdata  <= 0;
        end
        else begin
            if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin //nwrite swrite
                if (m_srio_req_en_r && (!m_srio_req_en_rr))  begin
                    val_ireq_tdata  <= header_beat;
                end 
                else if(val_ireq_tready) begin
                    if((pld_rem_size!=0) &&(val_ireq_tready && val_ireq_tvalid && val_ireq_tlast))begin  //next packet
                        val_ireq_tdata  <= header_beat;
                    end
                    else begin
                        //val_ireq_tdata  <= m_axi_rdata;
                        val_ireq_tdata  <= m_axi_rdata_sel;
                    end
                end
                else
                    val_ireq_tdata  <= val_ireq_tdata;
            end
            else if((m_srio_req_type == 8'h24 )  || (m_srio_req_type == 8'ha0)) begin  //nread  && doorbell 
                if (m_srio_req_en_r && (!m_srio_req_en_rr))  begin
                    val_ireq_tdata  <= header_beat;
                end 
                else  begin
                    if(val_ireq_tready && val_ireq_tlast) begin
                        if(pld_rem_size!=0)begin  //next packet
                            val_ireq_tdata  <= header_beat;
                        end
                        else begin
                            val_ireq_tdata  <= 32'h0;
                        end
                    end
                    else;
                end
            end
            else;
        end
    end

    //val_ireq_tvalid  
    always @(posedge log_clk) begin
        if (!log_rstn) begin 
            val_ireq_tvalid  <= 0;
        end
        else begin
            if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin //nwrite swrite
                if (m_srio_req_en_r && (!m_srio_req_en_rr))  begin
                    val_ireq_tvalid  <= 1'b1; // 1st header 
                end 
                else if(val_ireq_tready) begin
                    if((pld_rem_size!=0) &&(val_ireq_tready && val_ireq_tvalid && val_ireq_tlast))begin  //next packet
                        val_ireq_tvalid  <= 1'b1; //next header
                    end
                    else begin
                        val_ireq_tvalid  <= m_axi_rvalid;
                    end
                end
                else
                    val_ireq_tvalid  <= val_ireq_tvalid;
            end
            else if((m_srio_req_type == 8'h24 ) || (m_srio_req_type == 8'ha0)) begin  //nread  && doorbell 
                if (m_srio_req_en_r && (!m_srio_req_en_rr))  begin
                    val_ireq_tvalid  <= 1'b1;
                end 
                else  begin
                    if(val_ireq_tvalid && val_ireq_tready && val_ireq_tlast) begin
                        if(pld_rem_size!=0)begin  //next packet
                            val_ireq_tvalid  <= 1'b1;
                        end
                        else begin
                            val_ireq_tvalid  <= 1'b0;
                        end
                    end
                    else;
                end
            end
            else;
        end
    end

    //val_ireq_tlast
    always @(posedge log_clk) begin
        if (!log_rstn) begin
            val_ireq_tlast  <= 1'b0;
        end 
        else begin
            if(m_srio_req_type == 8'h54 || m_srio_req_type == 8'h55 || m_srio_req_type == 8'h60) begin //nwrite swrite
                if(val_ireq_tready) begin
                    val_ireq_tlast  <= m_axi_rlast && m_axi_rvalid; 
                end
                else;
            end
            else if((m_srio_req_type == 8'h24 )  || (m_srio_req_type == 8'ha0)) begin  //nread  && doorbell,header = last
                if (m_srio_req_en_r && (!m_srio_req_en_rr))  begin   //1st header
                    val_ireq_tlast  <= 1'b1;
                end 
                else  begin
                    if(val_ireq_tvalid && val_ireq_tready && val_ireq_tlast) begin
                        if(pld_rem_size!=0)begin  //next packet
                            val_ireq_tlast   <= 1'b1;
                        end
                        else begin
                            val_ireq_tlast   <= 1'b0;
                        end
                    end
                    else;
                end
            end
        end
    end

    //val_ireq_tkeep  
    assign val_ireq_tkeep  = 8'hFF;
    
    //val_ireq_tuser  
    assign val_ireq_tuser  = {src_id, dst_id};

     // w addr
 //   output  [4-1:0]                         m_axi_awid,
 //   output  [32-1:0]                        m_axi_awaddr,
 //   output                                  m_axi_awvalid,
 //   output  [7:0]                           m_axi_awlen,
 //   input                                   m_axi_awready,
 //   output  [1:0]                           m_axi_awburst,
 //   output     [2:0]                        m_axi_awsize,
 //  
 //   // w data
 //   output  [64-1:0]                        m_axi_wdata,
 //   output     [64/8-1:0]                   m_axi_wstrb,
 //   output                                  m_axi_wvalid,
 //   output                                  m_axi_wlast,
 //   input                                   m_axi_wready,

 //   // w response
 //   input [1:0]                             m_axi_bresp,
 //   input                                   m_axi_bvalid,
 //   output                                  m_axi_bready,

 //  //response from srio_gen2
 //  input             val_iresp_tvalid,
 //  output reg        val_iresp_tready,
 //  input             val_iresp_tlast,
 //  input      [63:0] val_iresp_tdata,
 //  input       [7:0] val_iresp_tkeep,
 //  input      [31:0] val_iresp_tuser,
    
	// frame end index
	reg iresp_eof;
    reg iresp_eof_r;

	always@(*) begin
		if(val_iresp_tlast && val_iresp_tvalid && val_iresp_tready)
			iresp_eof =1'b1;
		else if(val_iresp_tvalid)
			iresp_eof =1'b0;
		else
			iresp_eof = iresp_eof_r;
	end

    always @(posedge log_clk) begin
        if(!log_rstn) begin
			iresp_eof_r <=1'b1;
		end
		else begin
			iresp_eof_r <=iresp_eof;
		end
	end
    
	// response frame head data 
    reg [63:0]  hello_head_data;
    always @(posedge log_clk)  begin
       if(!log_rstn) 
           hello_head_data <= 64'h0;
       else begin
           if(iresp_eof_r && val_iresp_tvalid)
               hello_head_data <= val_iresp_tdata;
           else;
       end
    end

    //ftype ttype
    wire [3:0] response_ftype;
    wire [3:0] response_ttype;
    wire [7:0] response_type;

    assign  response_type =   hello_head_data[55:48]; 
    // nread length for m_axi_awlen to local mem
    reg [7:0] nread_len;
    reg [31:0] nread_rem_length;

    always @(posedge log_clk) begin
        if(!log_rstn) begin
            nread_rem_length <= 0;
            nread_len <= 0;
        end
        else begin
        	if(m_srio_req_en && (!m_srio_req_en_r)&&(m_srio_req_type == 8'h24)) begin   //calcuate 1st receive length
        	    if(m_srio_req_length[LEN:8]!=0) begin
        	        nread_rem_length <= m_srio_req_length - 256;
        	        nread_len <= 255;
        	    end
        	    else if(m_srio_req_length!=0) begin  
        	        nread_rem_length <= 0;
        	        nread_len <= (|m_srio_req_length[2:0])? m_srio_req_length[7:3]:(m_srio_req_length[7:3]-1); 
        	    end 
        	    else;
        	end
        	else begin
        	    if(val_iresp_tready && val_iresp_tvalid && val_iresp_tlast && (response_type == 8'hd8)) begin //calculate next receive length
        	        if(nread_rem_length[LEN:8]!=0) begin
        	            nread_rem_length <= nread_rem_length - 256;
        	            nread_len <= 255;
        	        end
        	        else if(nread_rem_length!=0) begin  
        	            nread_rem_length <= 0;
        	            nread_len <= (|nread_rem_length[2:0])? nread_rem_length[7:3]:(nread_rem_length[7:3]-1);
        	        end
        	    end
        	    else;
        	end
        end
    end

    //  nrd addr,for m_axi_awaddr to write to local mem
    reg [31:0] nread_addr;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            nread_addr <= 0;
        else  begin
        	if(m_srio_req_en && (!m_srio_req_en_r)&&(m_srio_req_type == 8'h24)) 
        	    //nread_addr <= m_srio_req_dst_addr;
        	    nread_addr <= m_srio_req_src_addr;
        	else if(val_iresp_tready && val_iresp_tvalid && val_iresp_tlast && (response_type == 8'hd8) &&(nread_rem_length !=0)) begin //calculate next receive length
        	    //nread_addr <= nread_addr + ({20'b0,(nread_len+9'b1)}<<3);  
        	    nread_addr <= nread_addr + ({24'b0,(nread_len+9'b1)});  
        	end
        	else; 
        end 
    end

    assign m_axi_awid = 4'h1;

    assign m_axi_awaddr = nread_addr;
    // m_axi_awvalid;
    reg m_axi_awvalid;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            m_axi_awvalid <= 0;
        end
        else begin
        	if(m_srio_req_en && (!m_srio_req_en_r)&&(m_srio_req_type == 8'h24)) begin   //calcuate 1st receive length
        	    if(m_srio_req_length[LEN:8]!=0) begin
        	        m_axi_awvalid <= 1;
        	    end
        	    else;
        	end
        	else if(val_iresp_tready && val_iresp_tvalid && val_iresp_tlast && (response_type == 8'hd8)) begin //calculate next receive length
        	    if(nread_rem_length[LEN:8]!=0) begin
        	        m_axi_awvalid <= 1;
        	    end
				else;
        	end
        	else if(m_axi_awready) begin 
        	    m_axi_awvalid <=1'b0;
        	end
        	else;
        end
    end
    // m_axi_awvalid;
	//
    assign val_iresp_tready =  iresp_eof_r ? m_axi_awready : m_axi_wready;  

    assign m_axi_awlen = (((nread_len +1) >>3) -1);
    assign m_axi_awburst = 2'h1;
    assign m_axi_awsize = 3'h3;  //8 byte

    // m_axi_wdata
	//
	integer i;
	reg [63:0] val_iresp_tdata_inv;
	always @(*) begin
		for(i =0 ;i <8; i=i+1) begin
			 val_iresp_tdata_inv[(i+1)*8-1 -:8] = val_iresp_tdata[(8-i)*8-1 -:8];
		end
	end

	assign    bit_inverse = srio_ip_rst[1];
	assign m_axi_wdata =  bit_inverse ? val_iresp_tdata_inv : 	val_iresp_tdata;


	//assign m_axi_wdata =  bit_inverse ? val_iresp_tdata_inv : 	val_iresp_tdata;


//assign m_axi_wdata =  bit_inverse ? {	val_iresp_tdata[ 7: 0],
//										val_iresp_tdata[15: 8]
//										val_iresp_tdata[23:16]
//										val_iresp_tdata[31:24]
//										val_iresp_tdata[63:56]
//										val_iresp_tdata[63:56]
//										val_iresp_tdata[63:56]
//										val_iresp_tdata[63:56]
//} : 
										;
    
    //m_axi_wvalid,
    always@(*) begin
        if((response_type == 8'hd8) && (!iresp_eof_r)) begin
            m_axi_wvalid =  val_iresp_tvalid; 
        end
        else
            m_axi_wvalid = 0; 
    end

    //m_axi_last,
    always@(*) begin
        if((response_type == 8'hd8) && (!iresp_eof_r)) begin
            m_axi_wlast =  val_iresp_tlast; 
        end
        else
            m_axi_wlast = 0; 
    end

    //m_axi_wstrb,
    assign m_axi_wstrb = 8'hffff;

    
    assign  m_axi_bready = 1'b1;
    wire [1:0] bresp;
    assign bresp = m_axi_bvalid ? m_axi_bresp : 2'b0;

	reg iresp_oof;
	
   	always @(posedge log_clk) begin
    	if(!log_rstn) begin
			iresp_oof <= 1'b1;
		end
		else begin
			if(val_iresp_tvalid && val_iresp_tready) begin
				if(val_iresp_tlast)
					iresp_oof <= 1'b1;
				else
					iresp_oof <= 1'b0;
			end
			else;
		end
	end
	
	reg [7:0] iresp_ftype_r;
	wire [7:0] iresp_ftype_c;
   	always @(posedge log_clk) begin
    	if(!log_rstn) begin
 			iresp_ftype_r <= 'h00;
		end
		else begin
 			iresp_ftype_r <=iresp_ftype_c;
		end
	end

	//assign iresp_ftype_c = (val_iresp_tvalid && val_iresp_tready && iresp_oof) ?  val_iresp_tdata[55:48] : 'h0;
	assign iresp_ftype_c = (val_iresp_tvalid && val_iresp_tready && iresp_oof) ?  val_iresp_tdata[55:48] : iresp_ftype_r;
   	always @(posedge log_clk) begin
    	if(!log_rstn) begin
			ireq_done <= 1'b0;
		end
		else begin
			if((!m_srio_req_en_rr) && m_srio_req_en_r ) begin
				ireq_done <= 1'b0;
			end
			else if(((m_srio_req_type == 'ha0) ||(pld_rem_size == 0)) && (val_ireq_tvalid && val_ireq_tready && val_ireq_tlast)) begin
				ireq_done <= 1'b1;
			end
			else;
		end
	end

	reg [63:0] iresp_cnt;
   	always @(posedge log_clk) begin
    	if(!log_rstn) begin
			iresp_cnt <= 'h0;
		end
		else begin
			if((!m_srio_req_en_rr) && m_srio_req_en_r ) begin
				iresp_cnt <= 'h0;
			end
			else if((val_iresp_tvalid && val_iresp_tready && val_iresp_tlast) && (((m_srio_req_type == 'h24) && (iresp_ftype_r == 'hd8)) || ((m_srio_req_type == 'h55) && (iresp_ftype_c == 'hd0)))) begin
				iresp_cnt <= iresp_cnt + 1'b1;
			end
			else;
		end
	end

   	always @(posedge log_clk) begin
    	if(!log_rstn) begin
			iresp_done <= 1'b0;
		end
		else begin
			if(( !m_srio_req_en_rr) && m_srio_req_en_r ) begin
				iresp_done <= 1'b0;
			end
			else if((iresp_cnt >= ( m_srio_req_length>>8)) || (((m_srio_req_type == 'h54) ||(m_srio_req_type == 'ha0)) && ireq_done)) begin // 256 bytes
				iresp_done <= 1'b1;
			end
			else;
		end
	end

endmodule
