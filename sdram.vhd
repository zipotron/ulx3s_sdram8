-- BASED on sdram controller implementation for the MiST board adaptation
-- of Luddes NES core
-- http://code.google.com/p/mist-board/
-- Harbaum <till@harbaum.org> 

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- This SDRAM controller provides a symmetric 32-bit synchronous read/write
-- interface for a 16Mx16-bit SDRAM chip (e.g. AS4C16M16SA-6TCN, IS42S16400F,
-- etc.).
entity sdram is
  generic (
    -- clock frequency (in MHz)
    --
    -- This value must be provided, as it is used to calculate the number of
    -- clock cycles required for the other timing values.
    -- CLK_FREQ : real := 25.0;

    CHIP_DATA_WIDTH : natural := 8;

    -- SDRAM interface
    SDRAM_ADDR_WIDTH : natural := 13;
    SDRAM_DATA_WIDTH : natural := 16
  );
  port (
    	-- interface to the MT48LC16M16 chip
    sd_data_in: in std_logic_vector(SDRAM_DATA_WIDTH-1 downto 0);
    sd_data_out: out std_logic_vector(SDRAM_DATA_WIDTH-1 downto 0);
	sd_addr: out std_logic_vector(SDRAM_ADDR_WIDTH-1 downto 0);
	sd_dqm: out std_logic_vector(1 downto 0);
	sd_ba: out std_logic_vector(1 downto 0);
	sd_cs: out std_logic;
	sd_we: out std_logic;
	sd_ras: out std_logic;
	sd_cas: out std_logic;

	-- cpu/chipset interface
	init: in std_logic;
	clk: in std_logic;
	clkref: in std_logic;
	we_out: out std_logic;
	
	addrA: in std_logic_vector(24 downto 0);
	weA: in std_logic;
	dinA: in std_logic_vector(CHIP_DATA_WIDTH-1 downto 0);
	oeA: in std_logic;
	doutA: out std_logic_vector(CHIP_DATA_WIDTH-1 downto 0);

	addrB: in std_logic_vector(24 downto 0);
	weB: in std_logic;
	dinB: in std_logic_vector(CHIP_DATA_WIDTH-1 downto 0);
	oeB: in std_logic;
	doutB: out std_logic_vector(CHIP_DATA_WIDTH-1 downto 0)
  );
end sdram;

architecture arch of sdram is

  constant RASCAS_DELAY	    : std_logic_vector(2 downto 0) := "010"; --tRCD=20ns -> 2 cycles@85MHz
  constant BURST_LENGTH	    : std_logic_vector(2 downto 0) := "000"; -- 000=1, 001=2, 010=4, 011=8
  constant ACCESS_TYPE		: std_logic := '0'; -- 0=sequential, 1=interleaved
  constant CAS_LATENCY	    : std_logic_vector(2 downto 0) := "011"; -- 2/3 allowed
  constant OP_MODE	    : std_logic_vector(1 downto 0) := "00"; -- only 00 (standard operation) allowed
  constant NO_WRITE_BURST		: std_logic := '1'; -- 0= write burst enabled, 1=only single access write
  constant MODE	    : std_logic_vector(12 downto 0) := "000" & NO_WRITE_BURST & OP_MODE & CAS_LATENCY & ACCESS_TYPE & BURST_LENGTH;

 ---------------------------------------------------------------------
 ------------------------ cycle state machine ------------------------
 ---------------------------------------------------------------------

  constant STATE_FIRST	    : std_logic_vector(2 downto 0) := "000"; -- first state in cycle
  constant STATE_CMD_START	    : std_logic_vector(2 downto 0) := "001"; -- state in which a new command can be started
  constant STATE_CMD_CONT	    : std_logic_vector(2 downto 0) := std_logic_vector(unsigned(STATE_CMD_START) + unsigned(RASCAS_DELAY)); -- 3 command can be continued
  constant STATE_CMD_READ	    : std_logic_vector(2 downto 0) := std_logic_vector(unsigned(STATE_CMD_CONT) + unsigned(CAS_LATENCY) + 1); -- 6 read state
  constant STATE_LAST	    : std_logic_vector(2 downto 0) := "111"; -- last state in cycle
  
  -- all possible commands
  constant CMD_INHIBIT	    	: std_logic_vector(3 downto 0) := "1111";
  constant CMD_NOP	    		: std_logic_vector(3 downto 0) := "0111";
  constant CMD_ACTIVE	    	: std_logic_vector(3 downto 0) := "0011";
  constant CMD_READ	    		: std_logic_vector(3 downto 0) := "0101";
  constant CMD_WRITE	    	: std_logic_vector(3 downto 0) := "0100";
  constant CMD_BURST_TERMINATE	: std_logic_vector(3 downto 0) := "0110";
  constant CMD_PRECHARGE	    : std_logic_vector(3 downto 0) := "0010";
  constant CMD_AUTO_REFRESH	    : std_logic_vector(3 downto 0) := "0001";
  constant CMD_LOAD_MODE	    : std_logic_vector(3 downto 0) := "0000";
  
  
  signal clkref_last   : std_logic;
  signal q    : std_logic_vector(2 downto 0);
  signal sd_cmd    : std_logic_vector(3 downto 0);
  signal reset    : std_logic_vector(16 downto 0);
  
  signal oe   : std_logic;
  signal addr    : std_logic_vector(24 downto 0);
  signal din    : std_logic_vector(7 downto 0);
  
  signal addr0   : std_logic;
  
  signal dout    : std_logic_vector(7 downto 0);
  
  signal reset_cmd    : std_logic_vector(3 downto 0);
  signal run_cmd    : std_logic_vector(3 downto 0);
  signal reset_addr    : std_logic_vector(12 downto 0);
  signal run_addr    : std_logic_vector(12 downto 0);
  
  signal we_out_aux   : std_logic;

begin
  -- SDRAM (state machine) clock is 85MHz. Synchronize this to systems 21.477 Mhz clock
  -- force counter to pass state LAST->FIRST exactly after the rising edge of clkref
  process (clk)
  begin
    if rising_edge(clk) then
      clkref_last <= clkref;

	  q <= std_logic_vector(unsigned(q) + 1);
      if q = STATE_LAST then q<=STATE_FIRST;
      end if;
      if ((clkref_last = '0') and (clkref = '1')) then q <= std_logic_vector(unsigned(STATE_FIRST) + 1);
      end if;
    end if;
  end process;

---------------------------------------------------------------------
--------------------------- startup/reset ---------------------------
---------------------------------------------------------------------

  -- wait 1ms (85000 cycles) after FPGA config is done before going
  -- into normal operation. Initialize the ram in the last 16 reset cycles (cycles 15-0)
  process (clk)
  begin
    if rising_edge(clk) then
      --if(init)	reset <= 17'h14c08;
	  if init = '1' then	reset <= "00000000000011111"; -- hex "1f"
	  
	  elsif((q = STATE_LAST) and (reset /= "00000000000000000")) then
		reset <= std_logic_vector(unsigned(reset) - 1);
      end if;
    end if;
  end process;
  
  oe <= oeA when clkref = '1' else oeB;
  addr <= addrA when clkref = '1' else addrB;
  din <= dinA when clkref = '1' else dinB;
  we_out_aux <= weA when clkref = '1' else weB;
  we_out <= we_out_aux;
  
  process (clk)
  begin
    if rising_edge(clk) then
      if((q = "001") and (oe = '1')) then addr0 <= addr(0);
      end if;
    end if;
  end process;
  
  dout <= sd_data_in(7 downto 0) when addr0 = '1' else sd_data_in(15 downto 8);
  
  process (clk)
  begin
    if rising_edge(clk) then
      if(q = STATE_CMD_READ) then
		if(oeA = '1' and  clkref = '1') then doutA <= dout;
		end if;
		if(oeB = '1' and clkref = '0') then doutB <= dout;
		end if;
	  end if;
    end if;
  end process;
  
  reset_cmd <= CMD_PRECHARGE when ((q = STATE_CMD_START) and (unsigned(reset) = 13)) else
			   CMD_LOAD_MODE when ((q = STATE_CMD_START) and (unsigned(reset) =  2)) else CMD_INHIBIT;
  
  run_cmd <= CMD_ACTIVE when ((we_out_aux = '1' or oe = '1') and (q = STATE_CMD_START)) else
			 CMD_WRITE when ( we_out_aux = '1' and (q = STATE_CMD_CONT )) else
			 CMD_READ when (we_out_aux = '0' and oe = '1' and (q = STATE_CMD_CONT )) else
			 CMD_AUTO_REFRESH when (we_out_aux = '0' and oe = '0' and (q = STATE_CMD_START)) else
			 CMD_INHIBIT;
  
  sd_cmd <= reset_cmd when reset /= "00000000000000000" else run_cmd;
  reset_addr <= "0010000000000" when unsigned(reset) = 13 else MODE;
  run_addr <= addr(21 downto 9) when q = STATE_CMD_START else "0010" & addr(24) & addr(8 downto 1);
  sd_data_out <= din & din when we_out_aux = '1' else "0000000000000000";
  sd_addr <= reset_addr when reset /= "00000000000000000" else run_addr;
  sd_ba <= "00" when reset /= "00000000000000000" else addr(23 downto 22);
  sd_dqm <= addr(0) & not addr(0) when we_out_aux = '1' else "00";
  
  sd_cs  <= sd_cmd(3);
  sd_ras <= sd_cmd(2);
  sd_cas <= sd_cmd(1);
  sd_we  <= sd_cmd(0);

end architecture arch;
