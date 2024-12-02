/***************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  *          http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  ***************************************************************************************/

package device

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.amba.axi4.{AXI4MasterNode, AXI4SlaveNode, AXI4MasterPortParameters, AXI4SlavePortParameters, AXI4Parameters}
import freechips.rocketchip.diplomacy.{AddressSet, LazyModule}
import xs.utils.{OneHot, ParallelPriorityMux, HoldUnless}
import freechips.rocketchip.diplomacy.BundleBridgeNexus.fillN

trait IOPMParameters {
  val BaseAddr = 0x47030000L
  val EntryOffset = 0x1100
  val RRIDNum = 4
  val KEntries = 2
  val Impid = 0

  def lgAlign = 2
}

trait IOPMPConst extends IOPMParameters {
  val EntryNum = RRIDNum * KEntries
  // 
  val PMPOffBits = 2
  val PlatformGrain = 2
  val lgMaxSize = 3
  val IOPMPAddrBits = 37
}

class AXI4IOPMP
(
  address: Seq[AddressSet],
  m_params: AXI4MasterPortParameters,
  s_params: AXI4SlavePortParameters
)(implicit p: Parameters)
  extends AXI4SlaveModule(address, executable = true, _extra = new Bundle {
    val interrupt = Output(Bool())
  })
{
  val out_node = AXI4MasterNode(Seq(m_params))
  val in_node = AXI4SlaveNode(Seq(s_params))

  override lazy val module = new AXI4SlaveModuleImp(this) {
    val (check_out, check_in_edge) = out_node.out.head
    val (check_in, check_out_edge) = in_node.in.head

    check_out <> check_in

    val iopmp = Module(new IOPMP)
    // IOPMP cfg READ
    // iopmp.io.cfg_en := in.ar.fire || in.aw.fire
    in.r.bits.data := iopmp.io.cfg_rdata

    // IOPMP cfg WRITE
    iopmp.io.cfg_wen := in.aw.fire
    iopmp.io.cfg_addr := waddr
    iopmp.io.cfg_wdata := in.w.bits.data
    iopmp.io.cfg_wmask := in.w.bits.strb

    // check read
    // val deny = RegInit(false.B)

    val s_chk_idle :: s_chk_rdata :: s_chk_wdata :: s_chk_wresp :: Nil = Enum(4)
    val chk_state = RegInit(s_chk_idle)

    switch(chk_state){
      is(s_chk_idle){
        when(check_in.ar.fire){
          chk_state := s_chk_rdata
        }
        when(check_in.aw.fire){
          chk_state := s_chk_wdata
        }
      }
      is(s_chk_rdata){
        when(check_in.r.fire && check_in.r.bits.last){
          chk_state := s_chk_idle
        }
      }
      is(s_chk_wdata){
        when(check_in.w.fire && check_in.w.bits.last){
          chk_state := s_chk_wresp
        }
      }
      is(s_chk_wresp){
        when(check_in.b.fire){
          chk_state := s_chk_idle
        }
      }
    }

    check_in.ar.ready := (state === s_chk_idle) && check_out.ar.ready
    check_in.aw.ready := (state === s_chk_idle) &&
      check_out.aw.ready && !check_in.ar.fire
    
    val deny_reg = RegInit(false.B)
    val deny = Mux(check_in.ar.fire || check_in.aw.fire, iopmp.io.check_deny,
      Mux(iopmp.io.check_deny, true.B, deny_reg))
    deny_reg := deny

    val rid = HoldUnless(check_in.ar.bits.id, check_in.ar.fire)
    val rsize = HoldUnless(check_in.ar.bits.size, check_in.ar.fire)

    val check_r_addr = RegInit(check_in.ar.bits.addr)
    when(check_in.ar.fire){
      check_r_addr := check_in.ar.bits.addr & (~((1.U << rsize)-1.U))
    }.elsewhen(check_in.r.fire){
      check_r_addr := check_r_addr + (1.U << rsize)
    }

    when(deny) {
      check_in.r.bits.data := 0.U
      check_in.r.bits.resp := AXI4Parameters.RESP_SLVERR
    }

    iopmp.io.check_rrid := 0.U
    iopmp.io.check_addr := 0.U
    iopmp.io.check_addr_write := false.B
    iopmp.io.check_size := 0.U

    when(check_in.ar.fire || chk_state === s_chk_rdata) {
      iopmp.io.check_rrid := rid
      iopmp.io.check_addr := Mux(check_in.ar.fire, check_in.ar.bits.addr, check_r_addr)
      iopmp.io.check_addr_write := false.B
      iopmp.io.check_size := 1.U << rsize
    }

    // check write
    val wid = HoldUnless(check_in.aw.bits.id, check_in.aw.fire)
    val wsize = HoldUnless(check_in.aw.bits.size, check_in.aw.fire)

    val check_w_addr = RegInit(check_in.aw.bits.addr)
    when(check_in.aw.fire){
      check_w_addr := check_in.aw.bits.addr & (~((1.U << wsize)-1.U))
    }.elsewhen(check_in.w.fire){
      check_w_addr := check_w_addr + (1.U << wsize)
    }

    when(deny) {
      check_in.w.bits.strb := 0.U
      check_in.b.bits.resp := AXI4Parameters.RESP_SLVERR
    }

    when(check_in.aw.fire || chk_state === s_chk_wdata) {
      iopmp.io.check_rrid := wid
      iopmp.io.check_addr := Mux(check_in.aw.fire, check_in.aw.bits.addr, check_w_addr)
      iopmp.io.check_addr_write := true.B
      iopmp.io.check_size := 1.U << wsize
    }

    dontTouch(io.extra.get.interrupt)
    io.extra.get.interrupt := iopmp.io.interrupt
  }
}

class IOPMPCfg extends Bundle with IOPMPConst {
  val rsv = UInt(25.W)
  val siwe = Bool()
  val sire = Bool()
  val a = UInt(2.W)
  val x = Bool()
  val w = Bool()
  val r = Bool()

  def off = a === 0.U
  def tor = a === 1.U
  def na4 = a === 2.U
  def napot = a === 3.U
  def off_tor = !a(1)
  def na4_napot = a(1)
}

class IOPMPBase extends Bundle with IOPMPConst {
  private val user_cfg = UInt(32.W)
  val cfg = new IOPMPCfg
  val addrh = UInt(32.W)
  val addr = UInt(32.W)
  // val addr = UInt(64.W)
}

class IOPMPEntry extends IOPMPBase with IOPMPMatchMethod {
  // val mask = UInt(IOPMPAddrBits.W) 
  val mask = UInt(64.W) 
}

class IOPMP extends Module with IOPMPConst {
  val io = IO(new Bundle {
    // val cfg_en = Input(Bool())
    val cfg_wen = Input(Bool())
    val cfg_addr = Input(UInt(64.W))
    val cfg_wdata = Input(UInt(64.W))
    val cfg_wmask = Input(UInt(8.W))
    val cfg_rdata = Output(UInt(64.W))

    val check_rrid = Input(UInt(16.W))
    val check_addr = Input(UInt(IOPMPAddrBits.W))
    val check_addr_write = Input(Bool())
    val check_size = Input(UInt(log2Ceil(lgMaxSize+1).W))
    val check_deny = Output(Bool())

    // val enable = Output(Bool())
    val interrupt = Output(Bool())
  })

  val deny = WireInit(false.B)
  // val suppress_interrupt = WireInit(false.B)
  // val check_no_hit = WireInit(false.B)
  // val check_id = 0.U(log2Up(KEntries).W)

  // CSR Define

  class Hwcfg0Struct extends Bundle {
    val enable = Bool()
     val dc = UInt(31.W)
  }

  class Hwcfg1Struct extends Bundle {
    val entry_num = UInt(16.W)
    val rrid_num = UInt(16.W)
  }

  class EntrylckStruct extends Bundle {
     val rsv = UInt(15.W)
    val f = UInt(16.W)
    val l = Bool()
  }

  class ErrCfgStruct extends Bundle {
     val dc = UInt(25.W)
    val rwe = Bool()
    val rre = Bool()
     val ixe = Bool()
    val iwe = Bool()
    val ire = Bool()
    val ie = Bool()
    val l = Bool()
  }

  class ErrReqinfoStruct extends Bundle {
     val dc = UInt(25.W)
    val etype = UInt(3.W)
     val rsv1 = Bool()
    val ttype = UInt(2.W)
    val v = Bool()
  }

  class ErrReqidStruct extends Bundle {
    val eid = UInt(16.W)
    val rrid = UInt(16.W)
  }

  val version = WireInit(9.U(32.W))
  val implementation = WireInit(Impid.U(32.W))
  val hwcfg0_reg = RegInit("h01004014".U(32.W))
  val hwcfg1 = WireInit(((EntryNum << 16)+RRIDNum).U(32.W))
  val entryoffset = WireInit(EntryOffset.U(32.W))
  val mdcfglck = WireInit(1.U(32.W))
  val entrylck_reg = RegInit(0.U(32.W))
  val err_cfg_reg = RegInit(0.U(32.W))
  val err_reqinfo_reg = RegInit(0.U(32.W))
  val err_reqaddr = RegInit(0.U(32.W))
  val err_reqaddrh = RegInit(0.U(32.W))
  val err_reqid_reg = RegInit(0.U(32.W))
  val mdcfg0 = WireInit(KEntries.U(32.W))

  val hwcfg0 = hwcfg0_reg.asTypeOf(new Hwcfg0Struct)
  val entrylck = entrylck_reg.asTypeOf(new EntrylckStruct)
  val err_cfg = err_cfg_reg.asTypeOf(new ErrCfgStruct)
  val err_reqinfo = err_reqinfo_reg.asTypeOf(new ErrReqinfoStruct)
  val err_reqid = err_reqid_reg.asTypeOf(new ErrReqidStruct)


  var iopmp_mapping = Map(
    MaskedRegMap(0x0, version),
    MaskedRegMap(0x4, implementation),
    MaskedRegMap(0x8, hwcfg0_reg, wmask = "h8000".U(32.W), wfn = x => {
      x | hwcfg0.asUInt
    }),
    MaskedRegMap(0xc, hwcfg1),
    MaskedRegMap(0x14, entryoffset),
    MaskedRegMap(0x48, mdcfglck),
    MaskedRegMap(0x4c, entrylck_reg, wmask = MaskedRegMap.WritableMask, wfn = x => {
      val xf = x.asTypeOf(new EntrylckStruct).f
      // when ( !entrylck.l ) {
      //   Cat(Mux(xf > entrylck.f, x(31,1), entrylck.asUInt(31,1)), x(0))
      // }.otherwise {
      //   entrylck.asUInt
      // }
      Mux(entrylck.l, entrylck.asUInt, Mux(xf >= entrylck.f, x, entrylck.asUInt))
    }),
    MaskedRegMap(0x60, err_cfg_reg, wmask = "h000f".U(32.W), wfn = x => {
      Mux(err_cfg.l, err_cfg.asUInt, x)
    }),
    MaskedRegMap(0x64, err_reqinfo_reg, wmask = "h0001".U(32.W), wfn = x => {
      Cat(err_reqinfo.asUInt(31, 1), Mux(x(0), false.B, err_reqinfo.v))
    }),
    MaskedRegMap(0x68, err_reqaddr),
    MaskedRegMap(0x6c, err_reqaddrh),
    MaskedRegMap(0x70, err_reqid_reg),
    MaskedRegMap(0x800, mdcfg0)
  )

  // val iopmp_entrys = RegInit(VecInit.fill(KEntries, RRIDNum)(
  //   0.U(128.W).asTypeOf(new IOPMPEntry)
  // ))
  val entry_addr = RegInit(VecInit.fill(EntryNum)(0.U(32.W)))
  val entry_addrh = RegInit(VecInit.fill(EntryNum)(0.U(32.W)))
  val entry_cfg = RegInit(VecInit.fill(EntryNum)(0.U(32.W)))
  val entry_mask = RegInit(VecInit.fill(EntryNum)(0.U((64).W)))

  val iopmp_entrys = WireInit(VecInit.fill(RRIDNum, KEntries)(
    0.U(192.W).asTypeOf(new IOPMPEntry)
  ))

  iopmp_entrys.flatten.zipWithIndex.map{case (entry, i) =>
    entry.addr := entry_addr(i)
    entry.addrh := entry_addrh(i)
    entry.cfg := entry_cfg(i).asTypeOf(new IOPMPCfg)
    entry.mask := entry_mask(i)

    iopmp_mapping = iopmp_mapping ++ Map(
      MaskedRegMap(EntryOffset + i*16, entry_addr(i),
        wmask = MaskedRegMap.WritableMask, wfn = x => {
          val addr = WireInit(entry_addr(i))
          when(i.U >= entrylck.f){
            entry_mask(i) := entry.match_mask(Cat(entry_addrh(i), x))
            addr := x
          }
          addr
        }),
      MaskedRegMap(EntryOffset + i*16 + 0x4, entry_addrh(i),
        wmask = MaskedRegMap.WritableMask, wfn = x => {
          val addr = WireInit(entry_addrh(i))
          when(i.U >= entrylck.f){
            entry_mask(i) := entry.match_mask(Cat(x, entry_addr(i)))
            addr := x
          }
          addr
        }),
      MaskedRegMap(EntryOffset + i*16 + 0x8, entry_cfg(i),
        wmask = "h007b".U(32.W), wfn = x => {
          val cfg = WireInit(entry_cfg(i))
          when(i.U >= entrylck.f){
            // entry_mask(i) := entry.match_mask(x.asTypeOf(new IOPMPCfg),
            //   Cat(entry_addrh(i), entry_addr(i)))
            cfg := x
          }
          cfg
        })
    )
  }

  val checker = IOPMPChecker(
    Mux1H(io.check_rrid, iopmp_entrys.toSeq),
    io.check_addr,
    io.check_addr_write,
    io.check_size,
    deny
  )

  // interrupt
  when( !err_reqinfo.v ) {
    when(err_cfg.ie && deny && !checker.io.suppress_interrupt (
        (err_cfg.ire && !io.check_addr_write) || 
        (err_cfg.iwe && io.check_addr_write))) {

      val new_err_reqinfo = WireInit(err_reqinfo)
      val new_err_reqid = WireInit(err_reqid)

      new_err_reqinfo.v := true.B
      new_err_reqinfo.ttype := Mux(io.check_addr_write, 2.U, 1.U)
      new_err_reqinfo.etype := MuxCase(0.U, Seq(
        (io.check_rrid >= RRIDNum.U) -> 6.U,
        checker.io.no_hit -> 5.U,
        !io.check_addr_write -> 1.U,
        io.check_addr_write -> 2.U
      ))
      err_reqinfo_reg := new_err_reqinfo.asUInt

      err_reqaddr := io.check_addr(31,0)
      err_reqaddrh := io.check_addr >> 32.U

      new_err_reqid.rrid := io.check_rrid
      new_err_reqid.eid := checker.io.id << io.check_rrid
      err_reqid_reg := new_err_reqid.asUInt
    }
  }.otherwise {
    // err_reqinfo.etype := 0.U
  }

  io.interrupt := Mux(!hwcfg0.enable, false.B, err_reqinfo.v)
  io.check_deny := Mux(!hwcfg0.enable, false.B, deny)
  // io.enable := hwcfg0.enable

  // split 64bits bus to 32bits
  val offset_addr = io.cfg_addr - BaseAddr.U
  val bus_select = offset_addr(2)
  val bus_wdata = WireInit(VecInit.fill(2)(0.U(32.W)))
  val bus_wmask = WireInit(VecInit.fill(2)(0.U(4.W)))
  val bus_rdata = WireInit(VecInit.fill(2)(0.U(32.W)))
  bus_wdata(1) := io.cfg_wdata(63, 32)
  bus_wdata(0) := io.cfg_wdata(31, 0)
  bus_wmask(1) := io.cfg_wmask(7, 4)
  bus_wmask(0) := io.cfg_wmask(3, 0)
  MaskedRegMap.generate(iopmp_mapping, offset_addr(63, 2) ## 0.U(2.W),
    bus_rdata(bus_select), io.cfg_wen, bus_wdata(bus_select),
    convertMask(bus_wmask(bus_select)))
  
  io.cfg_rdata := Cat(bus_rdata(1), bus_rdata(0))

  def convertMask(in: UInt): UInt = {
    val bits = in.asBools.map { bit =>
      Mux(bit, "hFF".U(8.W), 0.U(8.W))
    }
    Cat(bits)
  }
}

class IOPMPChecker extends Module with IOPMPConst {
  val io = IO(new Bundle {
    val iopmp_entry = Input(Vec(KEntries, new IOPMPEntry))

    val check_addr = Input(UInt(IOPMPAddrBits.W))
    val check_addr_write = Input(Bool())
    val check_size = Input(UInt(log2Ceil(lgMaxSize+1).W))
    val check_deny = Output(Bool())

    val suppress_interrupt = Output(Bool())
    val no_hit = Output(Bool())
    val id = Output(UInt(log2Up(KEntries).W))
  })
  val iopmps = io.iopmp_entry

  val iopmp0 = WireInit(0.U.asTypeOf(new IOPMPEntry))
  iopmp0.cfg.r := 0.U
  iopmp0.cfg.w := 0.U

  val match_vec = Wire(Vec(iopmps.size, Bool()))
  val cfg_vec = Wire(Vec(iopmps.size, new IOPMPEntry))

  iopmps.zip(iopmp0 +: iopmps.take(iopmps.size-1)).zipWithIndex.foreach{
    case ((iopmp, prev_iopmp), i) =>
      val is_match = iopmp.is_match(io.check_addr, io.check_size,
        lgMaxSize, prev_iopmp)
      val aligned = iopmp.aligned(io.check_addr, io.check_size,
        lgMaxSize, prev_iopmp)

      val cur = WireInit(iopmp)
      cur.cfg.r := aligned && iopmp.cfg.r
      cur.cfg.w := aligned && iopmp.cfg.w

      match_vec(i) := is_match
      cfg_vec(i) := cur
  }
  // match_vec(iopmps.size) := true.B
  // cfg_vec(iopmps.size) := iopmp0

  val mch = ParallelPriorityMux(match_vec, cfg_vec)

  io.check_deny := Mux(match_vec.asUInt === 0.U, true.B,
    Mux(io.check_addr_write, mch.cfg.w, mch.cfg.r))
  io.suppress_interrupt := Mux(io.check_addr_write, mch.cfg.siwe, mch.cfg.sire)
  io.no_hit := match_vec.asUInt === 0.U
  io.id := OHToUInt(match_vec)
}

object IOPMPChecker {
  def apply(
    iopmp_entry: Vec[IOPMPEntry],
    check_addr: UInt,
    check_addr_write: Bool,
    check_size: UInt,
    check_deny: Bool,
    // suppress_interrupt: Bool
  ) = {
    val checker = Module(new IOPMPChecker)
    checker.io.iopmp_entry := iopmp_entry
    checker.io.check_addr := check_addr
    checker.io.check_addr_write := check_addr_write
    checker.io.check_size := check_size
    check_deny := checker.io.check_deny
    // checker.io.suppress_interrupt := suppress_interrupt
    checker
  }
}

// trait PMPMethod extends IOPMPConst {
//   def computeMask = {
//     val base = Cat(addr, cfg.a(0)) | ((pmpGranularity - 1).U >> lgAlign)
//     Cat(base & ~(base + 1.U), ((1 << lgAlign) - 1).U)
//   }
// }

// copy from xs
object MaskedRegMap {
  import xs.utils.{MaskData, LookupTree}
  def Unwritable = null
  def NoSideEffect: UInt => UInt = (x=>x)
  def WritableMask = "h1111".U(32.W)
  def ReadableMask = "h1111".U(32.W)
  def UnwritableMask = 0.U(32.W)
  def apply(addr: Int, reg: UInt,
            wmask: UInt = UnwritableMask, wfn: UInt => UInt = Unwritable,
            rmask: UInt = ReadableMask, rfn: UInt => UInt = x=>x
           ): (Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)) = (addr, (reg, wmask, wfn, rmask, rfn))
  def generate(mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)], raddr: UInt, rdata: UInt,
    waddr: UInt, wen: Bool, wdata: UInt, wmask: UInt):Unit = {
    val chiselMapping = mapping.map { case (a, (r, wm, w, rm, rfn)) => (a.U, r, wm, w, rm, rfn) }
    rdata := LookupTree(raddr, chiselMapping.map { case (a, r, _, _, rm, rfn) => (a, rfn(r & rm)) })
    val wdata_reg = RegEnable(wdata, wen)
    chiselMapping.foreach { case (a, r, wm, w, _, _) =>
      if (w != null) {
        val wen_reg = wen && waddr === a
        when (wen_reg) { r := w(MaskData(r, wdata_reg, wm & wmask)) }
      }
    }
  }
  def generate(mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)], addr: UInt, rdata: UInt,
    wen: Bool, wdata: UInt, wmask: UInt):Unit = generate(mapping, addr, rdata, addr, wen, wdata, wmask)
}

// copy from xs
trait IOPMPMatchMethod extends IOPMPConst { this: IOPMPEntry =>
  /** compare_addr is used to compare with input addr */
  def compare_addr: UInt = ((Cat(addrh, addr) << PMPOffBits) & ~(((1 << PlatformGrain) - 1).U(IOPMPAddrBits.W))).asUInt

  /** size and maxSize are all log2 Size
   * for dtlb, the maxSize is bPMXLEN which is 8
   * for itlb and ptw, the maxSize is log2(512) ?
   * but we may only need the 64 bytes? how to prevent the bugs?
   * TODO: handle the special case that itlb & ptw & dcache access wider size than PMXLEN
   */
  def is_match(paddr: UInt, lgSize: UInt, lgMaxSize: Int, last_pmp: IOPMPEntry): Bool = {
    Mux(cfg.na4_napot, napotMatch(paddr, lgSize, lgMaxSize),
      Mux(cfg.tor, torMatch(paddr, lgSize, lgMaxSize, last_pmp), false.B))
  }

  /** generate match mask to help match in napot mode */
  def match_mask(cfg: IOPMPCfg, paddr: UInt): UInt = {
    val match_mask_c_addr = Cat(paddr, cfg.a(0)) | (((1 << PlatformGrain) - 1) >> PMPOffBits).U
    Cat(match_mask_c_addr & ~(match_mask_c_addr + 1.U), ((1 << PMPOffBits) - 1).U)
    // match_mask(cfg, paddr)
  }

  def match_mask(paddr: UInt): UInt = {
    match_mask(cfg, paddr)
  }

  def boundMatch(paddr: UInt, lgSize: UInt, lgMaxSize: Int): Bool = {
    if (lgMaxSize <= PlatformGrain) {
      (paddr < compare_addr)
    } else {
      val highLess = (paddr >> lgMaxSize) < (compare_addr >> lgMaxSize)
      val highEqual = (paddr >> lgMaxSize) === (compare_addr >> lgMaxSize)
      val lowLess = (paddr(lgMaxSize-1, 0) | OneHot.UIntToOH1(lgSize, lgMaxSize))  < compare_addr(lgMaxSize-1, 0)
      highLess || (highEqual && lowLess)
    }
  }

  def lowerBoundMatch(paddr: UInt, lgSize: UInt, lgMaxSize: Int): Bool = {
    !boundMatch(paddr, lgSize, lgMaxSize)
  }

  def higherBoundMatch(paddr: UInt, lgMaxSize: Int) = {
    boundMatch(paddr, 0.U, lgMaxSize)
  }

  def torMatch(paddr: UInt, lgSize: UInt, lgMaxSize: Int, last_pmp: IOPMPEntry): Bool = {
    last_pmp.lowerBoundMatch(paddr, lgSize, lgMaxSize) && higherBoundMatch(paddr, lgMaxSize)
  }

  def unmaskEqual(a: UInt, b: UInt, m: UInt) = {
    (a & ~m) === (b & ~m)
  }

  def napotMatch(paddr: UInt, lgSize: UInt, lgMaxSize: Int) = {
    if (lgMaxSize <= PlatformGrain) {
      unmaskEqual(paddr, compare_addr, mask)
    } else {
      val lowMask = mask | OneHot.UIntToOH1(lgSize, lgMaxSize)
      val highMatch = unmaskEqual(paddr >> lgMaxSize, compare_addr >> lgMaxSize, mask >> lgMaxSize)
      val lowMatch = unmaskEqual(paddr(lgMaxSize-1, 0), compare_addr(lgMaxSize-1, 0), lowMask(lgMaxSize-1, 0))
      highMatch && lowMatch
    }
  }

  def aligned(paddr: UInt, lgSize: UInt, lgMaxSize: Int, last: IOPMPEntry) = {
    if (lgMaxSize <= PlatformGrain) {
      true.B
    } else {
      val lowBitsMask = OneHot.UIntToOH1(lgSize, lgMaxSize)
      val lowerBound = ((paddr >> lgMaxSize) === (last.compare_addr >> lgMaxSize)) &&
        ((~paddr(lgMaxSize-1, 0) & last.compare_addr(lgMaxSize-1, 0)) =/= 0.U)
      val upperBound = ((paddr >> lgMaxSize) === (compare_addr >> lgMaxSize)) &&
        ((compare_addr(lgMaxSize-1, 0) & (paddr(lgMaxSize-1, 0) | lowBitsMask)) =/= 0.U)
      val torAligned = !(lowerBound || upperBound)
      val napotAligned = (lowBitsMask & ~mask(lgMaxSize-1, 0)) === 0.U
      Mux(cfg.na4_napot, napotAligned, torAligned)
    }
  }
}

