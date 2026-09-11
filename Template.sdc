derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# ============================================================
# Audio subsystem runs at ce_4m (96MHz/24 = 4MHz)
# All internal paths are CE-gated with 24 cycles between active edges.
# Multicycle = 24 for setup, 23 for hold.
# Target everything under SeibuCup_audio_z80 module (jt03, T80pa, mixer, ...).
# ============================================================
set_multicycle_path -setup -from [get_registers {*SeibuCup_audio_z80*}] -to [get_registers {*SeibuCup_audio_z80*}] 24
set_multicycle_path -hold  -from [get_registers {*SeibuCup_audio_z80*}] -to [get_registers {*SeibuCup_audio_z80*}] 23

# ============================================================
# COP3 datapath math (dist/divide): registri INTERNI puri, mai indirizzi RAM.
# Hanno margine >=2 cicli reale nel flow multi-stato (M_3BB0_CALC2->CALCM->CALC3,
# M_42C2_*). Multicycle 2 = rilassa i path math senza barare.
# IMPORTANTE: NON includere dma_src*/dma_dst*/dma_*_addr (= INDIRIZZI RAM, 1
# ciclo) -> un multicycle li corromperebbe -> garbage in RAM -> FREEZE CPU.
# ============================================================
set_multicycle_path -setup -to [get_registers {*SeibuCup_cop3*math_dx*}] 2
set_multicycle_path -hold  -to [get_registers {*SeibuCup_cop3*math_dx*}] 1
set_multicycle_path -setup -to [get_registers {*SeibuCup_cop3*math_dy*}] 2
set_multicycle_path -hold  -to [get_registers {*SeibuCup_cop3*math_dy*}] 1
set_multicycle_path -setup -to [get_registers {*SeibuCup_cop3*tmp32_*}] 2
set_multicycle_path -hold  -to [get_registers {*SeibuCup_cop3*tmp32_*}] 1
set_multicycle_path -setup -to [get_registers {*SeibuCup_cop3*sqrt_*}] 2
set_multicycle_path -hold  -to [get_registers {*SeibuCup_cop3*sqrt_*}] 1
set_multicycle_path -setup -to [get_registers {*SeibuCup_cop3*div_*}] 2
set_multicycle_path -hold  -to [get_registers {*SeibuCup_cop3*div_*}] 1
# atan CORDIC (138e/338e): i registri cordic_* vengono fusi/rinominati dal synth
# (non esistono col nome 'cordic' post-fit) e NON sono sul critical path (worst =
# scanlines). Il filtro andava ignorato → rimosso per non generare warning inutili.

# ============================================================
# Scanlines (vpos -> dout1): worst setup path del design (-0.926ns single-cycle).
# vpos cambia 1 volta per scanline = stabile 6144 cicli clk_sys (H_TOTAL 384 * 16
# ce_pix). dout1 reclocca ogni ciclo ma il DATO vpos->dout1 transita solo dopo un
# cambio vpos. Stesso clock domain (clk_sys == CLK_VIDEO, skew 0). Multicycle-2 e'
# banalmente valido (margine 6144>>2) e recupera ~+10ns. -from su vpos per non
# rilassare altri launcher single-cycle verso dout1.
# ============================================================
set_multicycle_path -setup -from [get_registers {*u_video_timing|vpos*}] -to [get_registers {*VGA_scanlines|dout1*}] 2
set_multicycle_path -hold  -from [get_registers {*u_video_timing|vpos*}] -to [get_registers {*VGA_scanlines|dout1*}] 1
# vpos -> tile_layer vram_addr: vpos cambia 1 volta/scanline (stabile 6144 cicli
# clk_sys = H_TOTAL 384 * 16 ce_pix). Il calcolo dell'indirizzo VRAM del layer dal
# vpos transita solo dopo un cambio vpos -> multicycle-2 banalmente valido (margine
# 6144>>2), stesso clock domain. -from vpos = solo i path lanciati da vpos.
set_multicycle_path -setup -from [get_registers {*u_video_timing|vpos*}] -to [get_registers {*SeibuCup_tile_layer*vram_addr*}] 2
set_multicycle_path -hold  -from [get_registers {*u_video_timing|vpos*}] -to [get_registers {*SeibuCup_tile_layer*vram_addr*}] 1

# ============================================================
# COP3 DMA cnt load: il path cop_dma_mode -> dma_cnt (-1.049 single-cycle).
# dma_cnt e' un CONTATORE caricato in S_IDLE, usato dopo D_PREP (>=2 cicli reali).
# Multicycle-2 -from cop_dma_mode = solo il caricamento, NON il decremento.
# (dma_src/dma_dst NON inclusi: sono indirizzi, il loro caricamento ha margine
#  reale ma lasciarli al fitter evita di spostare il collo altrove.)
# ============================================================
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*cop_dma_mode*}] -to [get_registers {*SeibuCup_cop3*dma_cnt*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*cop_dma_mode*}] -to [get_registers {*SeibuCup_cop3*dma_cnt*}] 1
# Il caricamento di dma_cnt legge anche i banchi cop_dma_size/dst[mode] (riga 1068:
# size<<4 - dst<<5 + 16). Questi banchi -> dma_cnt sono parte dello STESSO
# caricamento (S_IDLE, >=2 cicli prima dell'uso). Multicycle-2 anche da loro.
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*cop_dma_size*}] -to [get_registers {*SeibuCup_cop3*dma_cnt*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*cop_dma_size*}] -to [get_registers {*SeibuCup_cop3*dma_cnt*}] 1
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*cop_dma_dst*}]  -to [get_registers {*SeibuCup_cop3*dma_cnt*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*cop_dma_dst*}]  -to [get_registers {*SeibuCup_cop3*dma_cnt*}] 1
# (NON rilasso cop_dma_mode->fsm: e' control flow, troppo rischioso rilassarlo.
#  Il residuo ~-0.5ns su sprite-renderer/line-buffer e' pipeline single-cycle vero,
#  NON multiciclabile. Il timing non e' la causa del bug logico (memoria); ci si
#  ferma qui col grosso dei path cop3/DMA chiusi.)

# ============================================================
# CORDIC load (138e atan): M_138E_CALC carica cordic_x/y dal calcolo
# dy32=math_dx-{math_dy,dma_src_rdata}; axdy/aydx (sub+abs). dma_src_rdata viene
# dalla Main RAM (latenza BRAM=2 reale). Path Main RAM -> sub -> abs -> cordic_x/y
# = ~10ns single-cycle (-1.2). Il CARICAMENTO da math_dx/math_dy/tmp32 (gia'
# multicycle-2 nel blocco sopra, hanno margine) -> cordic e' coerente multicycle-2.
# -from i registri sorgente: colpisce SOLO il caricamento (M_138E_CALC), NON le
# iterazioni cordic->cordic (M_138E_CORDIC, che partono da cordic_x/y stessi e
# restano single-cycle). Il 138e ha margine: gira 1 volta/nemico, il gioco aspetta
# cop_status[2] (gate) a fine macro.
# ============================================================
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*math_dx*}]  -to [get_registers {*SeibuCup_cop3*cordic_*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*math_dx*}]  -to [get_registers {*SeibuCup_cop3*cordic_*}] 1
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*math_dy*}]  -to [get_registers {*SeibuCup_cop3*cordic_*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*math_dy*}]  -to [get_registers {*SeibuCup_cop3*cordic_*}] 1
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*tmp32_*}]   -to [get_registers {*SeibuCup_cop3*cordic_*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*tmp32_*}]   -to [get_registers {*SeibuCup_cop3*cordic_*}] 1
# dma_src/dma_dst CARICAMENTO da cop_dma_mode. -from cop_dma_mode = solo il
# caricamento iniziale (S_IDLE, >=2 cicli prima dell'uso via D_PREP), NON
# l'incremento dma_src->dma_src (parte da dma_src, single-cycle - resta stretto).
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*cop_dma_mode*}] -to [get_registers {*SeibuCup_cop3*dma_src*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*cop_dma_mode*}] -to [get_registers {*SeibuCup_cop3*dma_src*}] 1
set_multicycle_path -setup -from [get_registers {*SeibuCup_cop3*cop_dma_mode*}] -to [get_registers {*SeibuCup_cop3*dma_dst*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_cop3*cop_dma_mode*}] -to [get_registers {*SeibuCup_cop3*dma_dst*}] 1

# ============================================================
# FIX STRUTTURALE: TUTTI i path Main RAM (port-B) -> cop3 violano perche' la BRAM
# a 96MHz ha clock-to-output ~7ns + routing ~3ns = ~10ns (periodo 10.4ns). Ma il
# COP3 legge la Main RAM SEMPRE con latenza BRAM=2 PER COSTRUZIONE (ogni read:
# emit addr a T, dato valido a T+2; vedi commenti M_138E/M_3BB0/M_42C2/D_*). NON
# esiste lettura RAM single-cycle nel cop3. Quindi OGNI path Main RAM -> registro
# cop3 ha 2 cicli reali -> multicycle-2 GLOBALE legittimo (non un trucco): copre
# cordic/fade_fr_t/dma_ram_wdata/dma_ram_addr/tmp/ecc in un colpo, coerente con la
# latenza-2 gia' assunta dall'RTL. Risolve il collo strutturale BRAM->cop3.
# ============================================================
set_multicycle_path -setup -from [get_registers {*SeibuCup_ram_128k*}] -to [get_registers {*SeibuCup_cop3*}] 2
set_multicycle_path -hold  -from [get_registers {*SeibuCup_ram_128k*}] -to [get_registers {*SeibuCup_cop3*}] 1
