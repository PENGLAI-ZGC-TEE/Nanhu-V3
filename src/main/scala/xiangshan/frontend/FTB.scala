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

package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils._
import xs.utils.mbist.MBISTPipeline
import xs.utils.perf.HasPerfLogging
import xs.utils.sram.SRAMTemplate

import scala.{Tuple2 => &}


trait FTBParams extends HasXSParameter with HasBPUConst {
  val numEntries = FtbSize
  val numWays    = FtbWays
  val numSets    = numEntries/numWays // 512
  val tagSize    = 20

  

  val TAR_STAT_SZ = 2
  def TAR_FIT = 0.U(TAR_STAT_SZ.W)
  def TAR_OVF = 1.U(TAR_STAT_SZ.W)
  def TAR_UDF = 2.U(TAR_STAT_SZ.W)

  def BR_OFFSET_LEN = 12
  def JMP_OFFSET_LEN = 20
}

class FtbSlot(val offsetLen: Int, val subOffsetLen: Option[Int] = None)(implicit p: Parameters) extends XSBundle with FTBParams {
  if (subOffsetLen.isDefined) {
    require(subOffsetLen.get <= offsetLen)
  }
  val offset  = UInt(log2Ceil(PredictWidth).W)
  val lower   = UInt(offsetLen.W)
  val tarStat = UInt(TAR_STAT_SZ.W)
  val sharing = Bool()
  val valid   = Bool()

  def setLowerStatByTarget(pc: UInt, target: UInt, isShare: Boolean) = {
    def getTargetStatByHigher(pc_higher: UInt, target_higher: UInt) =
      Mux(target_higher > pc_higher, TAR_OVF,
        Mux(target_higher < pc_higher, TAR_UDF, TAR_FIT))
    def getLowerByTarget(target: UInt, offsetLen: Int) = target(offsetLen, 1)
    val offLen = if (isShare) this.subOffsetLen.get else this.offsetLen
    val pc_higher = pc(VAddrBits-1, offLen+1)
    val target_higher = target(VAddrBits-1, offLen+1)
    val stat = getTargetStatByHigher(pc_higher, target_higher)
    val lower = ZeroExt(getLowerByTarget(target, offLen), this.offsetLen)
    this.lower := lower
    this.tarStat := stat
    this.sharing := isShare.B
  }

  def getTarget(pc: UInt, last_stage: Option[Tuple2[UInt, Bool]] = None) = {
    def getTarget(offLen: Int)(pc: UInt, lower: UInt, stat: UInt,
      last_stage: Option[Tuple2[UInt, Bool]] = None) = {
      val h = pc(VAddrBits-1, offLen+1)
      val higher = Wire(UInt((VAddrBits-offLen-1).W))
      val higher_plus_one = Wire(UInt((VAddrBits-offLen-1).W))
      val higher_minus_one = Wire(UInt((VAddrBits-offLen-1).W))
      if (last_stage.isDefined) {
        val last_stage_pc = last_stage.get._1
        val last_stage_pc_h = last_stage_pc(VAddrBits-1, offLen+1)
        val stage_en = last_stage.get._2
        higher := RegEnable(last_stage_pc_h, stage_en)
        higher_plus_one := RegEnable(last_stage_pc_h+1.U, stage_en)
        higher_minus_one := RegEnable(last_stage_pc_h-1.U, stage_en)
      } else {
        higher := h
        higher_plus_one := h + 1.U
        higher_minus_one := h - 1.U
      }
      val target =
        Cat(
          Mux1H(Seq(
            (stat === TAR_OVF, higher_plus_one),
            (stat === TAR_UDF, higher_minus_one),
            (stat === TAR_FIT, higher),
          )),
          lower(offLen-1, 0), 0.U(1.W)
        )
      require(target.getWidth == VAddrBits)
      require(offLen != 0)
      target
    }
    if (subOffsetLen.isDefined)
      Mux(sharing,
        getTarget(subOffsetLen.get)(pc, lower, tarStat, last_stage),
        getTarget(offsetLen)(pc, lower, tarStat, last_stage)
      )
    else
      getTarget(offsetLen)(pc, lower, tarStat, last_stage)
  }
  def fromAnotherSlot(that: FtbSlot) = {
    require(
      this.offsetLen > that.offsetLen && this.subOffsetLen.map(_ == that.offsetLen).getOrElse(true) ||
      this.offsetLen == that.offsetLen
    )
    this.offset := that.offset
    this.tarStat := that.tarStat
    this.sharing := (this.offsetLen > that.offsetLen && that.offsetLen == this.subOffsetLen.get).B
    this.valid := that.valid
    this.lower := ZeroExt(that.lower, this.offsetLen)
  }
  
}

class FTBEntry(implicit p: Parameters) extends XSBundle with FTBParams with BPUUtils {
  
  
  val valid       = Bool()

  // val brSlots = Vec(numBrSlot, new FtbSlot(BR_OFFSET_LEN))

  val tailSlot = new FtbSlot(JMP_OFFSET_LEN, Some(BR_OFFSET_LEN))

  // Partial Fall-Through Address
  val pftAddr     = UInt(log2Up(PredictWidth).W)
  val carry       = Bool()

  val isCall      = Bool()
  val isRet       = Bool()
  val isJalr      = Bool()

  val last_may_be_rvi_call = Bool()

  val always_taken = Vec(numBr, Bool())

  def getSlotForBr(idx: Int): FtbSlot = {
    // require(idx <= numBr-1)
    // (idx, numBr) match {
    //   case (i, n) if i == n-1 => this.tailSlot
    //   case _ => this.brSlots(idx)
    // }
    this.tailSlot
  }
  def allSlotsForBr = {
    (0 until numBr).map(getSlotForBr(_))
  }
  def setByBrTarget(brIdx: Int, pc: UInt, target: UInt) = {
    val slot = getSlotForBr(brIdx)
    slot.setLowerStatByTarget(pc, target, brIdx == numBr-1)
  }
  def setByJmpTarget(pc: UInt, target: UInt) = {
    this.tailSlot.setLowerStatByTarget(pc, target, false)
  }

  def getTargetVec(pc: UInt, last_stage: Option[Tuple2[UInt, Bool]] = None) = {
    // VecInit((brSlots :+ tailSlot).map(_.getTarget(pc, last_stage)))
    VecInit((tailSlot).getTarget(pc, last_stage))
  }

  // def getOffsetVec = VecInit(brSlots.map(_.offset) :+ tailSlot.offset)
  def getOffsetVec = VecInit(tailSlot.offset)
  def isJal = !isJalr
  def getFallThrough(pc: UInt) = getFallThroughAddr(pc, carry, pftAddr)
  def hasBr(offset: UInt) =
    //brSlots.map{ s => s.valid && s.offset <= offset}.reduce(_||_) ||
    (tailSlot.valid && tailSlot.offset <= offset && tailSlot.sharing)

  def getBrMaskByOffset(offset: UInt) =
    //brSlots.map{ s => s.valid && s.offset <= offset } :+
    (tailSlot.valid && tailSlot.offset <= offset && tailSlot.sharing)
    
  def getBrRecordedVec(offset: UInt) = {
    VecInit(
      //brSlots.map(s => s.valid && s.offset === offset) :+
      (tailSlot.valid && tailSlot.offset === offset && tailSlot.sharing)
    )
  }
    
  def brIsSaved(offset: UInt) = getBrRecordedVec(offset).reduce(_||_)

  def brValids = {
    VecInit(
      //brSlots.map(_.valid) :+ (tailSlot.valid && tailSlot.sharing)
      (tailSlot.valid && tailSlot.sharing)
    )
  }

  def noEmptySlotForNewBr = {
    // VecInit(brSlots.map(_.valid) :+ tailSlot.valid).reduce(_&&_)
    VecInit(tailSlot.valid).reduce(_&&_)
  }

  def newBrCanNotInsert(offset: UInt) = {
    val lastSlotForBr = tailSlot
    lastSlotForBr.valid && lastSlotForBr.offset < offset
  }

  def jmpValid = {
    tailSlot.valid && !tailSlot.sharing
  }

  def brOffset = {
    // VecInit(brSlots.map(_.offset) :+ tailSlot.offset)
      VecInit(tailSlot.offset)
  }

  def display(cond: Bool): Unit = {
  }

}

class FTBEntryWithTag(implicit p: Parameters) extends XSBundle with FTBParams with BPUUtils {
  val entry = new FTBEntry
  val tag = UInt(tagSize.W)
  def display(cond: Bool): Unit = {
    entry.display(cond)
  }
}

class FTBMeta(implicit p: Parameters) extends XSBundle with FTBParams {
  val writeWay = UInt(log2Ceil(numWays).W)
  val hit = Bool()
  val fauFtbHit = if (EnableFauFTB) Some(Bool()) else None
  val pred_cycle = if (!env.FPGAPlatform) Some(UInt(64.W)) else None
}

object FTBMeta {
  def apply(writeWay: UInt, hit: Bool, fauhit: Bool, pred_cycle: UInt)(implicit p: Parameters): FTBMeta = {
    val e = Wire(new FTBMeta)
    e.writeWay := writeWay
    e.hit := hit
    e.fauFtbHit.map(_ := fauhit)
    e.pred_cycle.map(_ := pred_cycle)
    e
  }
}

// class UpdateQueueEntry(implicit p: Parameters) extends XSBundle with FTBParams {
//   val pc = UInt(VAddrBits.W)
//   val ftb_entry = new FTBEntry
//   val hit = Bool()
//   val hit_way = UInt(log2Ceil(numWays).W)
// }
//
// object UpdateQueueEntry {
//   def apply(pc: UInt, fe: FTBEntry, hit: Bool, hit_way: UInt)(implicit p: Parameters): UpdateQueueEntry = {
//     val e = Wire(new UpdateQueueEntry)
//     e.pc := pc
//     e.ftb_entry := fe
//     e.hit := hit
//     e.hit_way := hit_way
//     e
//   }
// }

class FTB(parentName:String = "Unknown")(implicit p: Parameters) extends BasePredictor with FTBParams with BPUUtils
  with HasCircularQueuePtrHelper with HasPerfEvents {
  override val meta_size = WireInit(0.U.asTypeOf(new FTBMeta)).getWidth

  val ftbAddr = new TableAddr(log2Up(numSets), 1)

  class FTBBank(val numSets: Int, val nWays: Int) extends XSModule with BPUUtils {
    val io = IO(new Bundle {
      val s1_fire = Input(Bool())

      // when ftb hit, read_hits.valid is true, and read_hits.bits is OH of hit way
      // when ftb not hit, read_hits.valid is false, and read_hits is OH of allocWay
      // val read_hits = Valid(Vec(numWays, Bool()))
      val req_pc = Flipped(DecoupledIO(UInt(VAddrBits.W)))
      val read_resp = Output(new FTBEntry)
      val read_hits = Valid(UInt(log2Ceil(numWays).W))

      val u_req_pc = Flipped(DecoupledIO(UInt(VAddrBits.W)))
      val update_hits = Valid(UInt(log2Ceil(numWays).W))
      val update_access = Input(Bool())

      val update_pc = Input(UInt(VAddrBits.W))
      val update_write_data = Flipped(Valid(new FTBEntryWithTag))
      val update_write_way = Input(UInt(log2Ceil(numWays).W))
      val update_write_alloc = Input(Bool())
    })

    // Extract holdRead logic to fix bug that update read override predict read result
    val ftb = Module(new SRAMTemplate(new FTBEntryWithTag, set = numSets, way = numWays, shouldReset = true, holdRead = false, singlePort = true,
      hasMbist = coreParams.hasMbist,
      hasShareBus = coreParams.hasShareBus,
      parentName = parentName
    ))
    val ftb_r_entries = ftb.io.r.resp.data.map(_.entry)

    val pred_rdata   = HoldUnless(ftb.io.r.resp.data, RegNext(io.req_pc.valid && !io.update_access))
    ftb.io.r.req.valid := io.req_pc.valid || io.u_req_pc.valid // io.s0_fire
    ftb.io.r.req.bits.setIdx := Mux(io.u_req_pc.valid, ftbAddr.getIdx(io.u_req_pc.bits), ftbAddr.getIdx(io.req_pc.bits)) // s0_idx

    assert(!(io.req_pc.valid && io.u_req_pc.valid))

    io.req_pc.ready := ftb.io.r.req.ready
    io.u_req_pc.ready := ftb.io.r.req.ready

    val req_tag = RegEnable(ftbAddr.getTag(io.req_pc.bits)(tagSize-1, 0), io.req_pc.valid)
    val req_idx = RegEnable(ftbAddr.getIdx(io.req_pc.bits), io.req_pc.valid)

    val u_req_tag = RegEnable(ftbAddr.getTag(io.u_req_pc.bits)(tagSize-1, 0), io.u_req_pc.valid)

    val read_entries = pred_rdata.map(_.entry)
    val read_tags    = pred_rdata.map(_.tag)

    val total_hits = VecInit((0 until numWays).map(b => read_tags(b) === req_tag && read_entries(b).valid && io.s1_fire))
    val hit = total_hits.reduce(_||_)
    // val hit_way_1h = VecInit(PriorityEncoderOH(total_hits))
    val hit_way = OHToUInt(total_hits)

    val u_total_hits = VecInit((0 until numWays).map(b =>
        ftb.io.r.resp.data(b).tag === u_req_tag && ftb.io.r.resp.data(b).entry.valid && RegNext(io.update_access)))
    val u_hit = u_total_hits.reduce(_||_)
    // val hit_way_1h = VecInit(PriorityEncoderOH(total_hits))
    val u_hit_way = OHToUInt(u_total_hits)

    // assert(PopCount(total_hits) === 1.U || PopCount(total_hits) === 0.U)
    // assert(PopCount(u_total_hits) === 1.U || PopCount(u_total_hits) === 0.U)
    for (n <- 1 to numWays) {
      XSPerfAccumulate(f"ftb_pred_${n}_way_hit", PopCount(total_hits) === n.U)
      XSPerfAccumulate(f"ftb_update_${n}_way_hit", PopCount(u_total_hits) === n.U)
    }

    val replacer = ReplacementPolicy.fromString(Some("setplru"), numWays, numSets)
    // val allocWriteWay = replacer.way(req_idx)

    val touch_set = Seq.fill(1)(Wire(UInt(log2Ceil(numSets).W)))
    val touch_way = Seq.fill(1)(Wire(Valid(UInt(log2Ceil(numWays).W))))

    val write_set = Wire(UInt(log2Ceil(numSets).W))
    val write_way = Wire(Valid(UInt(log2Ceil(numWays).W)))

    val read_set = Wire(UInt(log2Ceil(numSets).W))
    val read_way = Wire(Valid(UInt(log2Ceil(numWays).W)))

    read_set := req_idx
    read_way.valid := hit
    read_way.bits  := hit_way

    touch_set(0) := Mux(write_way.valid, write_set, read_set)

    touch_way(0).valid := write_way.valid || read_way.valid
    touch_way(0).bits := Mux(write_way.valid, write_way.bits, read_way.bits)

    replacer.access(touch_set, touch_way)

    def allocWay(valids: UInt, idx: UInt): UInt = {
      if (numWays > 1) {
        val w = Wire(UInt(log2Up(numWays).W))
        val valid = WireInit(valids.andR)
        w := Mux(valid, replacer.way(idx), PriorityEncoder(~valids))
        w
      } else {
        val w = WireInit(0.U(log2Up(numWays).W))
        w
      }
    }

    io.read_resp := Mux1H(total_hits, read_entries) // Mux1H
    io.read_hits.valid := hit
    io.read_hits.bits := hit_way

    io.update_hits.valid := u_hit
    io.update_hits.bits := u_hit_way

    // Update logic
    val u_valid = io.update_write_data.valid
    val u_data = io.update_write_data.bits
    val u_idx = ftbAddr.getIdx(io.update_pc)
    val allocWriteWay = allocWay(RegNext(VecInit(ftb_r_entries.map(_.valid))).asUInt, u_idx)
    val u_way = Mux(io.update_write_alloc, allocWriteWay, io.update_write_way)
    val u_mask = UIntToOH(u_way)

    for (i <- 0 until numWays) {
      XSPerfAccumulate(f"ftb_replace_way$i", u_valid && io.update_write_alloc && u_way === i.U)
      XSPerfAccumulate(f"ftb_replace_way${i}_has_empty", u_valid && io.update_write_alloc && !ftb_r_entries.map(_.valid).reduce(_&&_) && u_way === i.U)
      XSPerfAccumulate(f"ftb_hit_way$i", hit && !io.update_access && hit_way === i.U)
    }

    ftb.io.w.apply(u_valid, u_data, u_idx, u_mask)

    // for replacer
    write_set := u_idx
    write_way.valid := u_valid
    write_way.bits := Mux(io.update_write_alloc, allocWriteWay, io.update_write_way)

    // print hit entry info
    Mux1H(total_hits, ftb.io.r.resp.data).display(true.B)
  } // FTBBank

  val ftbBank = Module(new FTBBank(numSets, numWays))
  val mbistPipeline = if (coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistPipe")
  } else {
    None
  }

  ftbBank.io.req_pc.valid := io.s0_fire(dupForFtb)
  ftbBank.io.req_pc.bits := s0_pc_dup(dupForFtb)

  val btb_enable_dup = RegNext(dup(io.ctrl.btb_enable))
  val s2_ftb_entry_dup = io.s1_fire.map(f => RegEnable(ftbBank.io.read_resp, f))
  val s3_ftb_entry_dup = io.s2_fire.zip(s2_ftb_entry_dup).map {case (f, e) => RegEnable(e, f)}
  
  val s1_ftb_hit = ftbBank.io.read_hits.valid && io.ctrl.btb_enable
  val s1_uftb_hit_dup = io.in.bits.resp_in(0).s1.full_pred.map(_.hit)
  val s2_ftb_hit_dup = io.s1_fire.map(f => RegEnable(s1_ftb_hit, f))
  val s2_uftb_hit_dup =
    if (EnableFauFTB) {
      io.s1_fire.zip(s1_uftb_hit_dup).map {case (f, h) => RegEnable(h, f)}
    } else {
      s2_ftb_hit_dup
    }
  val s2_real_hit_dup = s2_ftb_hit_dup.zip(s2_uftb_hit_dup).map(tp => tp._1 || tp._2)
  val s3_hit_dup = io.s2_fire.zip(s2_real_hit_dup).map {case (f, h) => RegEnable(h, f)}
  val writeWay = ftbBank.io.read_hits.bits

  // io.out.bits.resp := RegEnable(io.in.bits.resp_in(0), 0.U.asTypeOf(new BranchPredictionResp), io.s1_fire)
  io.out := io.in.bits.resp_in(0)

  val s1_latch_call_is_rvc   = DontCare // TODO: modify when add RAS

  io.out.s2.full_pred.zip(s2_real_hit_dup).map {case (fp, h) => fp.hit := h}
  val s2_uftb_full_pred_dup = io.s1_fire.zip(io.in.bits.resp_in(0).s1.full_pred).map {case (f, fp) => RegEnable(fp, f)}
  for (full_pred & s2_ftb_entry & s2_pc & s1_pc & s1_fire & s2_uftb_full_pred & s2_hit & s2_uftb_hit <-
    io.out.s2.full_pred zip s2_ftb_entry_dup zip s2_pc_dup zip s1_pc_dup zip io.s1_fire zip s2_uftb_full_pred_dup zip
    s2_ftb_hit_dup zip s2_uftb_hit_dup) {
      if (EnableFauFTB) {
        // use uftb pred when ftb not hit but uftb hit
        when (!s2_hit && s2_uftb_hit) {
          full_pred := s2_uftb_full_pred
        }.otherwise {
          full_pred.fromFtbEntry(s2_ftb_entry, s2_pc, Some((s1_pc, s1_fire)))
        }
      } else {
        full_pred.fromFtbEntry(s2_ftb_entry, s2_pc, Some((s1_pc, s1_fire)))
      }
    }

  // s3 
  val s3_full_pred = io.s2_fire.zip(io.out.s2.full_pred).map {case (f, fp) => RegEnable(fp, f)}
  // br_taken_mask from SC in stage3 is covered here, will be recovered in always taken logic
  io.out.s3.full_pred := s3_full_pred

  val s3_fauftb_hit_ftb_miss = RegEnable(!s2_ftb_hit_dup(dupForFtb) && s2_uftb_hit_dup(dupForFtb), io.s2_fire(dupForFtb))
  io.out.last_stage_ftb_entry := Mux(s3_fauftb_hit_ftb_miss, io.in.bits.resp_in(0).last_stage_ftb_entry, s3_ftb_entry_dup(dupForFtb))
  io.out.last_stage_meta := RegEnable(RegEnable(FTBMeta(writeWay.asUInt, s1_ftb_hit, s1_uftb_hit_dup(dupForFtb), GTimer()).asUInt, io.s1_fire(dupForFtb)), io.s2_fire(dupForFtb))

  // always taken logic
  for (i <- 0 until numBr) {
    for (out_fp & in_fp & s2_hit & s2_ftb_entry <-
      io.out.s2.full_pred zip io.in.bits.resp_in(0).s2.full_pred zip s2_ftb_hit_dup zip s2_ftb_entry_dup)
      out_fp.br_taken_mask(i) := in_fp.br_taken_mask(i) || s2_hit && s2_ftb_entry.always_taken(i)
    for (out_fp & in_fp & s3_hit & s3_ftb_entry <-
      io.out.s3.full_pred zip io.in.bits.resp_in(0).s3.full_pred zip s3_hit_dup zip s3_ftb_entry_dup)
      out_fp.br_taken_mask(i) := in_fp.br_taken_mask(i) || s3_hit && s3_ftb_entry.always_taken(i)
  }

  // Update logic
  val u = io.update(dupForFtb)
  val update = u.bits

  val u_meta = update.meta.asTypeOf(new FTBMeta)
  // we do not update ftb on fauFtb hit and ftb miss
  val update_uftb_hit_ftb_miss = u_meta.fauFtbHit.getOrElse(false.B) && !u_meta.hit
  val u_valid = u.valid && !u.bits.old_entry && !(update_uftb_hit_ftb_miss)

  val updateDelay2 = Pipe(u, 2)
  val delay2_pc = updateDelay2.bits.pc
  val delay2_entry = updateDelay2.bits.ftb_entry

  
  val update_now = u_valid && u_meta.hit
  val update_need_read = u_valid && !u_meta.hit
  // stall one more cycle because we use a whole cycle to do update read tag hit
  io.s1_ready := ftbBank.io.req_pc.ready && !(update_need_read) && !RegNext(update_need_read)

  ftbBank.io.u_req_pc.valid := update_need_read
  ftbBank.io.u_req_pc.bits := update.pc



  val ftb_write = Wire(new FTBEntryWithTag)
  ftb_write.entry := Mux(update_now, update.ftb_entry, delay2_entry)
  ftb_write.tag   := ftbAddr.getTag(Mux(update_now, update.pc, delay2_pc))(tagSize-1, 0)

  val write_valid = update_now || DelayN(u_valid && !u_meta.hit, 2)

  ftbBank.io.update_write_data.valid := write_valid
  ftbBank.io.update_write_data.bits := ftb_write
  ftbBank.io.update_pc          := Mux(update_now, update.pc,       delay2_pc)
  ftbBank.io.update_write_way   := Mux(update_now, u_meta.writeWay, RegNext(ftbBank.io.update_hits.bits)) // use it one cycle later
  ftbBank.io.update_write_alloc := Mux(update_now, false.B,         RegNext(!ftbBank.io.update_hits.valid)) // use it one cycle later
  ftbBank.io.update_access := u_valid && !u_meta.hit
  ftbBank.io.s1_fire := io.s1_fire(dupForFtb)

  XSDebug("req_v=%b, req_pc=%x, ready=%b (resp at next cycle)\n", io.s0_fire(dupForFtb), s0_pc_dup(dupForFtb), ftbBank.io.req_pc.ready)
  XSDebug("s2_hit=%b, hit_way=%b\n", s2_ftb_hit_dup(dupForFtb), writeWay.asUInt)
  XSDebug("s2_br_taken_mask=%b, s2_real_taken_mask=%b\n",
    io.in.bits.resp_in(dupForFtb).s2.full_pred(dupForFtb).br_taken_mask.asUInt, io.out.s2.full_pred(dupForFtb).real_slot_taken_mask().asUInt)
  XSDebug("s2_target=%x\n", io.out.s2.target(dupForFtb))

  s2_ftb_entry_dup(dupForFtb).display(true.B)

  XSPerfAccumulate("ftb_read_hits", RegNext(io.s0_fire(dupForFtb)) && s1_ftb_hit)
  XSPerfAccumulate("ftb_read_misses", RegNext(io.s0_fire(dupForFtb)) && !s1_ftb_hit)

  XSPerfAccumulate("ftb_commit_hits", u.valid && u_meta.hit)
  XSPerfAccumulate("ftb_commit_misses", u.valid && !u_meta.hit)

  XSPerfAccumulate("ftb_update_req", u.valid)

  XSPerfAccumulate("ftb_update_ignored_old_entry", u.valid && u.bits.old_entry)
  XSPerfAccumulate("ftb_update_ignored_fauftb_hit", u.valid && update_uftb_hit_ftb_miss)
  XSPerfAccumulate("ftb_updated", u_valid)

  override val perfEvents = Seq(
    ("ftb_commit_hits            ", u.valid  &&  u_meta.hit),
    ("ftb_commit_misses          ", u.valid  && !u_meta.hit),
  )
  generatePerfEvent()
}
