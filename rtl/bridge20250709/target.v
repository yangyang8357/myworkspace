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


module target(
    //input
    input link_initialized,
    //input port_initialized,
    //input mode_1x,   
    input                                   m_axi_aclk,
    input                                   m_axi_aresetn,

    input             log_clk,
    input             log_rstn,
     // aw addr
    output  [4-1:0]                         m_axi_awid,
    output  [32-1:0]                        m_axi_awaddr,
    output  reg                             m_axi_awvalid,
    output  [7:0]                           m_axi_awlen,
    input                                   m_axi_awready,
    output  [1:0]                           m_axi_awburst,
    output     [2:0]                        m_axi_awsize,
    
    // w data
    output  [64-1:0]                        m_axi_wdata,
    output  [64/8-1:0]                      m_axi_wstrb,
    output    reg                           m_axi_wvalid,
    output    reg                           m_axi_wlast,
    input                                   m_axi_wready,
    
    // w response
    input [1:0]                             m_axi_bresp,
    input                                   m_axi_bvalid,
    output                                  m_axi_bready,
    
    // ar addr
    output     [2:0]                        m_axi_arsize,
    output  [1:0]                           m_axi_arburst,
    output  [31:0]                          m_axi_araddr,
    output  [4-1:0]                         m_axi_arid,
    output                                  m_axi_arvalid,
    output  [7:0]                           m_axi_arlen,
    input                                   m_axi_arready,
    
    //rd data
    input [63:0]                            m_axi_rdata,
    input [1:0]                             m_axi_rresp,
    input                                   m_axi_rvalid,
    input                                   m_axi_rlast,
    output            reg                   m_axi_rready,
    
    // write port
    input                       full,
    input                       almost_full,
 (*MARK_DEBUG = "true"*)    output          [63:0]      wr_data,
    output                      wr_en,

    // read port
    input                       empty,
    input                       almost_empty,
 (*MARK_DEBUG = "true"*)    input           [63:0]      rd_data,
    output                      rd_en,


    //target response
    output reg          val_tresp_tvalid,
    input               val_tresp_tready,
    output reg          val_tresp_tlast,
  (*MARK_DEBUG = "true"*)   output reg [63:0]   val_tresp_tdata,
    output      [7:0]   val_tresp_tkeep,
   (*MARK_DEBUG = "true"*)   output     [31:0]   val_tresp_tuser,
    

    //target request
    input               val_treq_tvalid,
    output reg          val_treq_tready,
    input               val_treq_tlast,
  (*MARK_DEBUG = "true"*)   input      [63:0]   val_treq_tdata,
    input       [7:0]   val_treq_tkeep,
    input      [31:0]   val_treq_tuser,
    
	// to initiator
	input				bit_inv,
	input 	[31:0] 		waddr,
    input 	[31:0] 		waddr_mask,
	input 	[31:0] 		raddr,
    input 	[31:0] 		raddr_mask,
	output  [31:0]		doorbell_info,
	output     			irq_target
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

    localparam [2:0] IDLE = 4'h0;
    localparam [2:0] RESP_HEAD = 4'h1;
    localparam [2:0] RESP_DATA = 4'h2;

//    localparam [7:0] NREAD      = 8'h24;
//    localparam [7:0] NWRITE     = 8'h54;
//    localparam [7:0] NWRITE_R   = 8'h55;
//    localparam [7:0] SWRITE     = 8'h60;
//    localparam [7:0] DOORBELL   = 8'ha0;
//    localparam [7:0] RESP       = 8'hd0;
//    localparam [7:0] RESP_DATA  = 8'hd8;
// regs

    //target request
    //  input             val_treq_tvalid,
    //  output reg        val_treq_tready,
    //  input             val_treq_tlast,
    //  input      [63:0] val_treq_tdata,
    //  input       [7:0] val_treq_tkeep,
    //  input      [31:0] val_treq_tuser
    //  input       [7:0] user_size,
    //  input      [63:0] user_data,
    

	reg [63:0]  m_axi_rdata_inv;
	wire [63:0]  m_axi_rdata_sel;
	integer i;
	always@(*) begin
		for(i =0 ;i <8; i=i+1) begin
			  m_axi_rdata_inv[(i+1)*8-1 -:8] =    m_axi_rdata[(8-i)*8-1 -:8];
		end
	end
	assign   m_axi_rdata_sel =  bit_inv ? m_axi_rdata_inv : m_axi_rdata;

    // out of frame index 
    reg treq_oof;
    // response frame head data 
    reg [63:0]  hello_head_data;
    always @(posedge log_clk)  begin
       	if(!log_rstn) 
           	hello_head_data <= 64'h0;
       	else begin
           	if(treq_oof && val_treq_tvalid)
                hello_head_data <= val_treq_tdata;
           	else if( val_treq_tvalid &&  val_treq_tready && val_treq_tlast)
           		hello_head_data <= 64'h0;
		 	else;  
       end
    end


    // start of frame  1 tick
    reg treq_sof;
    wire treq_sof_c;
    assign  treq_sof_c = val_treq_tready && val_treq_tvalid && treq_oof;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            treq_sof <= 1'b0;
        else begin
            //if( val_treq_tready && val_treq_tvalid && treq_oof) 
            if( val_treq_tready && val_treq_tvalid && treq_oof) 
                treq_sof <= 1'b1;
            else
                treq_sof <= 1'b0;
        end
    end

    //ftype ttype
    wire [3:0]  req_ftype;
    wire [3:0]  req_ttype;
    wire [7:0]  req_type;
    wire [7:0]  req_len;
    wire [31:0] req_addr;

    assign  req_ftype =  hello_head_data[55:52]; 
    assign  req_ttype =  hello_head_data[51:48]; 
    assign  req_addr =   hello_head_data[31:0];
    assign  req_len  =   hello_head_data[43:36];   

   // wire [3:0]  current_req_ftype;
    //wire [3:0]  current_req_ttype;

    //val_treq_tready,
    //// aw addr
    //output  [4-1:0]                         m_axi_awid,
    //output  [32-1:0]                        m_axi_awaddr,
    //output                                  m_axi_awvalid,
    //output  [7:0]                           m_axi_awlen,
    //input                                   m_axi_awready,
    //output  [1:0]                           m_axi_awburst,
    //output     [2:0]                        m_axi_awsize,
      // incoming packet fields
    wire  [7:0]     current_tid;
    wire  [3:0]     current_ftype;
    wire  [3:0]     current_ttype;
    wire  [7:0]     current_size;
    wire  [1:0]     current_prio;
    wire [33:0]     current_addr;
    wire [15:0]     current_srcid;
    //
    assign m_axi_awid = 4'h1;
    assign m_axi_awaddr =(( waddr & waddr_mask ) | ( req_addr & (~ waddr_mask )));

    //reg    m_axi_awvalid;
    reg treq_eof;
    reg treq_eof_r;

	always@(*) begin
		if(val_treq_tlast && val_treq_tvalid && val_treq_tready)
			treq_eof =1'b1;
		else if(val_treq_tvalid)
			treq_eof =1'b0;
		else
			treq_eof = treq_eof_r;
	end

    always @(posedge log_clk) begin
        if(!log_rstn) begin
			treq_eof_r <=1'b1;
		end
		else begin
			treq_eof_r <=treq_eof;
		end
	end
		
    //reg    m_axi_awvalid;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            m_axi_awvalid <= 1'b0;
        end 
        else begin
            //if(val_treq_tready && val_treq_tvalid && treq_oof) begin
            //if((!val_treq_tlast) && val_treq_tvalid && ((!treq_eof) && treq_eof_r)) begin
            if( val_treq_tvalid && (treq_eof_r)) begin
                if((current_ftype == NWRITE) || (current_ftype == SWRITE)) begin
                    m_axi_awvalid <= 1'b1;
                end
                else
                    m_axi_awvalid <= 1'b0;
            end
            else begin
                if(m_axi_awready)
                    m_axi_awvalid <= 1'b0;
                else
                    m_axi_awvalid <= m_axi_awvalid;
            end
        end
    end

	reg irq_target;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
			irq_target <= 1'b0;
		end
        else  begin
			if( val_treq_tvalid && treq_eof_r && (current_ftype ==  DOORB)) begin
				irq_target <= 1'b1;
			end
			else
				irq_target <= 1'b0;
		end
	end


	reg	[31:0] 	doorbell_info;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
			doorbell_info <= 1'b0;
		end
        else  begin
			if( val_treq_tvalid && treq_eof_r && (current_ftype ==  DOORB)) begin
				doorbell_info <= current_addr[31:0]; 
			end
			else;
		end
	end



    assign m_axi_awlen = (((req_len+1)>>3)-1);  
    assign m_axi_awburst = 2'h1;
    assign m_axi_awsize = 3'h3;  //8 byte

     // w data
    //output  [64-1:0]                        m_axi_wdata,
    //output  [64/8-1:0]                      m_axi_wstrb,
    //output                                  m_axi_wvalid,
    //output                                  m_axi_wlast,
    //input                                   m_axi_wready,

	reg [63:0]    val_treq_tdata_inv;
	always @(*) begin
		for(i =0 ;i <8; i=i+1) begin
	    	val_treq_tdata_inv[(i+1)*8-1 -:8] =   val_treq_tdata[(8-i)*8-1 -:8];
		end
	end
	assign m_axi_wdata =  bit_inv ?  val_treq_tdata_inv : val_treq_tdata;
    // m_axi_wdata
    // assign m_axi_wdata =  val_treq_tdata;
    
    //m_axi_wstrb,
    assign m_axi_wstrb = 8'hffff;

    reg m_axi_wvalid_r;
	reg m_axi_wready_r;
    reg m_axi_wlast_r;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            m_axi_wvalid_r <= 1'b0;
         	m_axi_wready_r <= 1'b0;
            m_axi_wlast_r <= 1'b0;
        end 
		else begin
            m_axi_wvalid_r  <= m_axi_wvalid;
           	m_axi_wready_r  <= m_axi_wready;
            m_axi_wlast_r <= m_axi_wlast;
		end
	end

    //m_axi_wvalid,
    always@(*) begin
        if(((req_ftype == NWRITE) || (req_ftype == SWRITE)) && (!treq_oof)) begin
            m_axi_wvalid =  val_treq_tvalid; 
        end
        else if(m_axi_wlast)
            m_axi_wvalid = 1; 
		else
            m_axi_wvalid = 0; 
    end


    //m_axi_wlast
    always@(*) begin
        if((req_ftype == NWRITE) || (req_ftype == SWRITE)) begin
            m_axi_wlast =  val_treq_tlast; 
        end
        else if(m_axi_wready_r)
            m_axi_wlast = 0; 
		else 
            m_axi_wlast = m_axi_wlast_r; 

    end

//    // w response
//    input [1:0]                             m_axi_bresp,
//    input                                   m_axi_bvalid,
//    output                                  m_axi_bready,
    assign  m_axi_bready = 1'b1;
    wire [1:0] bresp;
    assign bresp = m_axi_bvalid ? m_axi_bresp : 2'b0;

    //// r addr
    //output     [2:0]                        m_axi_arsize,
    //output  [1:0]                           m_axi_arburst,
    //output  [31:0]                          m_axi_araddr,
    //output  [4-1:0]                         m_axi_arid,
    //output                                  m_axi_arvalid,
    //output  [7:0]                           m_axi_arlen,
    //input                                   m_axi_arready,
    //

    assign m_axi_arsize = 3'h3; //8bytes
    assign m_axi_arburst = 2'h1; // increase mode
    //assign m_axi_araddr =  req_addr;
    assign m_axi_araddr =(( raddr & raddr_mask ) | ( current_addr[31:0] & (~raddr_mask)));
    assign m_axi_arid = 0; 
    //assign m_axi_arlen = (((req_len+1)>>3)-1);
    assign m_axi_arlen = (((current_size+1)>>3)-1);


	reg treq_sof;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
            treq_sof <= 1'b0;
		end
		else begin
            treq_sof <= (val_treq_tready && val_treq_tvalid && treq_oof);
		end
	end
 //  reg    m_axi_arvalid;

	reg	treq_nrd_sof;  //nread sof 
    always @(posedge log_clk) begin
        if(!log_rstn) begin
	   		treq_nrd_sof <=1'b0;
		end
		else begin
			if((val_treq_tready && val_treq_tvalid && val_treq_tlast))
				treq_nrd_sof <=1'b1;
			else
				treq_nrd_sof <=1'b0;
		end
	end

   	reg 	m_axi_arvalid_r;
    always @(posedge log_clk) begin
        if(!log_rstn) begin
	   	 	m_axi_arvalid_r	<=1'b0;
		end
		else begin
	   	 	m_axi_arvalid_r	<=m_axi_arvalid;

		end
	end

 	reg    m_axi_arvalid;
   	always @(*) begin
        if((!log_rstn) || (!link_initialized)) begin
    	    m_axi_arvalid = 1'b0;
		end
		else if(val_treq_tready && val_treq_tvalid && (current_ftype == NREAD) && treq_oof) begin
    		m_axi_arvalid = 1'b1;
    	end
		else if(treq_nrd_sof && (req_ftype == NREAD)) begin
    		m_axi_arvalid = 1'b0;
    	end
		else begin
    	    m_axi_arvalid =m_axi_arvalid_r;
		end
    	//else begin
    	//    if(m_axi_arready)
    	//        m_axi_arvalid = 1'b0;
    	//    else
    	//        m_axi_arvalid =m_axi_arvalid;
    	//end
   	end

    //treq out of frame
    reg treq_oof;
    always @(posedge log_clk) begin
        if (!log_rstn) begin
            treq_oof <= 1'b1;
        end 
        else begin
            if (val_treq_tready && val_treq_tvalid && val_treq_tlast) begin
                treq_oof <= 1'b1;
            //end else if (val_treq_tready && val_treq_tvalid) begin
            end else if (val_treq_tready && val_treq_tvalid) begin
                treq_oof <= 1'b0;
            end
            else;
        end
    end
 
  // wire  [3:0] current_ftype;
  // wire  [3:0] current_ttype;
  // wire  [7:0] current_size;

    reg generate_a_response;
    always @(posedge log_clk) begin 
        if (!log_rstn) begin
            generate_a_response <= 1'b0;
        end  begin
            if(treq_oof && val_treq_tready && val_treq_tvalid) begin
                generate_a_response <= (current_ftype == NREAD) ||
                              (current_ftype == DOORB) ||
                              (current_ftype == MESSG) ||
                              ((current_ftype == NWRITE) && (current_ttype == TNWR_R));
            end 
            else begin
                generate_a_response <= 1'b0;
            end
        end
    end

  // reg    val_treq_tready;
  	reg    val_treq_tready;
  	always @(*) begin 
  		if((!link_initialized) || (!log_rstn)) begin
  	        val_treq_tready = 0;
  		end
  		else if((current_ftype == DOORB) && (treq_oof && val_treq_tvalid && val_treq_tlast)) begin
  	        val_treq_tready = !almost_full;
  		end
  		else if((current_ftype == NREAD) && (treq_oof && val_treq_tvalid && val_treq_tlast)) begin
  	        val_treq_tready = (!almost_full) && m_axi_arready; 
  		end
  		//else if((current_ftype == NWRITE) && (val_treq_tvalid )) begin
  		else if((req_ftype == NWRITE) && (val_treq_tvalid )) begin
  			if(req_ttype == TNWR_R)
  	            val_treq_tready = (!almost_full) && m_axi_wready;
  			else
  	            val_treq_tready = m_axi_wready;
  		end
  		else  begin
  	        val_treq_tready = 1'b0;
  		end
  	end

    assign current_tid   = val_treq_tdata[63:56];
    assign current_ftype = val_treq_tdata[55:52];
    assign current_ttype = val_treq_tdata[51:48];
    assign current_size  = val_treq_tdata[43:36];
    assign current_prio  = val_treq_tdata[46:45] + 2'b01;
    assign current_addr  = val_treq_tdata[33:0];
    assign current_srcid = val_treq_tuser[31:16];

    // outgoing packet fields
    wire [15:0]     dest_id;
    wire [15:0]     src_id;
    wire  [7:0]     response_tid;
    wire  [3:0]     response_ftype;
    wire  [3:0]     response_ttype;
    wire  [7:0]     response_size;
    wire  [1:0]     response_prio;
    wire [63:0]     response_data_out_d;
    wire  [63:0] 	response_data_out; // upper 63:47 unused

	// rd_data_d
	//
	reg  [63:0] rd_data_d;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
 			rd_data_d <='b0;
		else 
 			rd_data_d <= rd_data; 
	end
	assign response_data_out    = rd_data_d;
    assign response_tid         = response_data_out[19:12];
    assign response_ftype       = response_data_out[11:8];
    assign response_size        = response_data_out[7:0];
    assign response_prio        = response_data_out[21:20];
    assign dest_id              = response_data_out[46:31];
    assign starting_read_addr   = {1'b0, response_data_out[29:22]};
    assign pull_from_store      = response_data_out[30];

//   // write port
    wire [63:0]response_data_in_d;  
    assign response_data_in_d = {17'h0, current_srcid, current_addr[23:16] == 8'h12, current_addr[10:3],current_prio, current_tid, current_ftype, current_size};
   
    reg [63:0]   response_data_in;                            
    always @ (posedge log_clk) begin
        if(!log_rstn) 
            response_data_in <= 64'h0;
        else
            response_data_in <= response_data_in_d;
    end
    
    assign wr_data = response_data_in;
    // tresp out of frame index 
    reg tresp_oof;   
    wire rd_en;
    //assign rd_en  = (( val_tresp_tvalid && val_tresp_tready  && val_tresp_tlast) || (treq_oof)) && (!empty);
    //assign rd_en  = (( val_tresp_tvalid && val_tresp_tready  && val_tresp_tlast) || (tresp_oof && (!empty)));
    reg rd_en_r;
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            rd_en_r <= 1'b0;
        else 
            rd_en_r <= rd_en;
    end

    wire wr_en;
    assign wr_en = generate_a_response  &&  (!full);

    // tresp out of frame index 
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            tresp_oof <= 1'b1;
        else begin
            if( val_tresp_tready && val_tresp_tvalid) begin
                if( val_tresp_tlast)
                    tresp_oof <= 1'b1;
                else
                    tresp_oof <= 1'b0;
            end
            else;
        end
    end

    reg  [2:0] resp_state_c;
    reg  [2:0] resp_state;

	assign rd_en = ((resp_state_c == RESP_HEAD) &&(resp_state != RESP_HEAD))? 1'b1 : 1'b0;

    always @(posedge log_clk)  begin
        if(!log_rstn) 
            resp_state <= 3'b0;
        else
            resp_state <= resp_state_c;
    end
    
        
    always @(*) begin
        resp_state_c = resp_state;
        case(resp_state) 
            IDLE : begin
    			if((tresp_oof && (!empty))) begin
    			//if(( val_tresp_tvalid && val_tresp_tready  && val_tresp_tlast) || (tresp_oof && (!empty))) begin
                //if(rd_en) begin
                    resp_state_c = RESP_HEAD;
                end
                else;
            end

            RESP_HEAD : begin //response 1 ready tick
                //if((response_ftype  == NREAD) && val_tresp_tready ) begin
                if(val_tresp_tready ) begin
                	if(response_ftype  == NREAD)begin 
                    	resp_state_c = RESP_DATA;
					end
					else
                    	resp_state_c = IDLE;
                end
                else begin //doorbell nwrite
                    resp_state_c = RESP_HEAD;

                end
            end

            RESP_DATA : begin  //response with data
                if(val_tresp_tlast && val_tresp_tvalid && val_tresp_tready) begin //transfer done
                    resp_state_c = IDLE;
                end
                else begin
                    resp_state_c = RESP_DATA;
                end
            end

            default: begin  
                resp_state_c = IDLE;
            end
        endcase
    end

    //val_tresp_tvalid 
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            val_tresp_tvalid <=1'b0;
        else begin
            if(resp_state == RESP_HEAD )  begin
                val_tresp_tvalid <=1'b1;
            end
            else if((resp_state == IDLE) || (resp_state_c == IDLE))  begin
                val_tresp_tvalid <= 1'b0;
			end
            else if(resp_state == RESP_DATA)  begin
                if(val_tresp_tready) 
                   val_tresp_tvalid <= m_axi_rvalid;
                else
                   val_tresp_tvalid <= val_tresp_tvalid;
            end
            else;
        end
    end


    //  val_tresp_tlast 
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            val_tresp_tlast <= 1'b0;
        else begin
            if((resp_state == RESP_HEAD) && (response_ftype  != NREAD))
                val_tresp_tlast <= 1'b1;
            else if(resp_state == RESP_DATA)
                if(val_tresp_tready)
                    val_tresp_tlast <=  m_axi_rlast;
                else
                    val_tresp_tlast <=  val_tresp_tlast;
            else
                val_tresp_tlast <= 1'b0;
        end
    end


    //val_tresp_tkeep  
    assign val_tresp_tkeep  = 8'hFF;


    //val_tresp_tuser  
   // assign val_tresp_tuser  = {response_tid, dest_id};
    assign val_tresp_tuser  = {16'h00ff, 16'h00ff};
    
    wire [63:0] resp_header_beat;
    wire [3:0]  resp_ttype;
    assign resp_ttype = ( response_ftype  == NREAD )? TWDATA : TNDATA;  // response with/without data

    //                            8          4'hd       4         1             2     1     8    1    1      2     32
    assign resp_header_beat  = {response_tid, RESP, resp_ttype, 1'b0, response_prio, 1'b0, 8'hff,1'b0,1'b0, 2'b0, 32'h0};

    //val_tresp_tdata 
    always @(posedge log_clk)  begin
        if(!log_rstn) 
            val_tresp_tdata <= 64'h0;
        else begin
            if(resp_state == RESP_HEAD )
                val_tresp_tdata <= resp_header_beat;
            else if(!m_axi_rready)  //keep 
                val_tresp_tdata <=val_tresp_tdata;
			else
                //val_tresp_tdata <= m_axi_rdata; m_axi_rdata_sel 
                val_tresp_tdata <= m_axi_rdata_sel; 
        end
    end

    //m_axi_rready,
    always @(*)  begin
        if(resp_state == IDLE)
            m_axi_rready = 0;
        else if( (resp_state_c == IDLE) && (resp_state == RESP_DATA) && m_axi_rlast)
            m_axi_rready = val_tresp_tready;
        else if( (resp_state_c != RESP_DATA) || (resp_state != RESP_DATA))
            m_axi_rready = 0;
        else if(resp_state == RESP_DATA)
            m_axi_rready = val_tresp_tready;
    end

    //target response
    //output reg          val_tresp_tvalid,
    //input               val_tresp_tready,
    //output reg          val_tresp_tlast,
    //output reg [63:0]   val_tresp_tdata,
    //output      [7:0]   val_tresp_tkeep,
    //output     [31:0]   val_tresp_tuser,
    //


    ////rd data
    //input [63:0]                            m_axi_rdata,
    //input [1:0]                             m_axi_rresp,
    //input                                   m_axi_rvalid,
    //input                                   m_axi_rlast,
    //output                                  m_axi_rready,
    
endmodule

