package xiangshan.vector.viwaitqueue

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import xiangshan.backend.execute.fu.FuOutput
import xiangshan.backend.rob.RobPtr
import xiangshan.{MicroOp, Redirect, XSBundle, XSModule}
import xiangshan.vector.HasVectorParameters
import xiangshan.vector.writeback.WbMergeBufferPtr

class VIWakeQueueEntry(implicit p: Parameters) extends XSBundle{
  val uop = new MicroOp
  val vtypeRdy = Bool()
  val robEnqueued = Bool()
  val mergeIdAlloc = Bool()
}
class ViwqWritePort(implicit p: Parameters) extends XSBundle{
  val wen = Input(Bool())
  val data = Input(new VIWakeQueueEntry)
  val addr = Input(UInt(log2Ceil(VIWaitQueueWidth).W))
}

class ViwqReadPort(implicit p: Parameters) extends XSBundle{
  val addr = Input(UInt(log2Ceil(VIWaitQueueWidth).W))
  val data = new VIWakeQueueEntry
}

class VmsIdAlloc(implicit p: Parameters) extends XSBundle{
  val en = Input(Bool())
  val data = Input(new WbMergeBufferPtr(VectorMergeBufferDepth))
  val addr = Input(UInt(log2Ceil(VIWaitQueueWidth).W))
}

class VIWakeQueueEntryUpdateNetwork(implicit p: Parameters) extends XSModule with HasVectorParameters{
  val io = IO(new Bundle{
    val enq = Input(Valid(new VIWakeQueueEntry))
    val entry = Input(new VIWakeQueueEntry)
    val robEnq = Input(Vec(RenameWidth, Valid(new RobPtr)))
    val vmsResp = Input(Valid(new WbMergeBufferPtr(VectorMergeBufferDepth)))
    val vtypeWb = Input(Valid(new FuOutput(XLEN)))
    val entryNext = Output(new VIWakeQueueEntry)
    val updateEnable = Output(Bool())
  })
  private val entryNext = WireInit(io.entry)
  when(io.enq.valid){
    entryNext := io.enq.bits
  }

  private val robEnqHit = io.robEnq.map(r => r.valid && r.bits === io.entry.uop.robIdx).reduce(_|_)
  when(io.enq.valid){
    entryNext.robEnqueued := io.enq.bits.robEnqueued
  }.elsewhen(robEnqHit){
    entryNext.robEnqueued := true.B
  }

  when(io.enq.valid){
    entryNext.mergeIdAlloc := io.enq.bits.mergeIdAlloc
    entryNext.uop.mergeIdx := DontCare
  }.elsewhen(io.vmsResp.valid){
    entryNext.mergeIdAlloc := true.B
    entryNext.uop.mergeIdx := io.vmsResp.bits
  }

  private val vtypeWbHit = io.vtypeWb.valid && io.vtypeWb.bits.uop.vtypeRegIdx === io.entry.uop.vtypeRegIdx
  when(io.enq.valid) {
    entryNext.vtypeRdy := io.enq.bits.vtypeRdy
    entryNext.uop.vCsrInfo := io.enq.bits.uop.vCsrInfo
  }.elsewhen(vtypeWbHit) {
    entryNext.robEnqueued := true.B
    //TODO: fill this
    entryNext.uop.vCsrInfo := DontCare
  }

  io.entryNext := entryNext
  io.updateEnable := io.enq.valid || vtypeWbHit || robEnqHit || io.vmsResp.valid
}

class VIWaitQueueArray(implicit p: Parameters) extends XSModule with HasVectorParameters{
  private val size = VIWaitQueueWidth
  val io = IO(new Bundle{
    val enq = Vec(VIDecodeWidth, new ViwqWritePort)
    val deq = new ViwqReadPort
    val robEnq = Input(Vec(RenameWidth, Valid(new RobPtr)))
    val vmsIdAllocte = Vec(VIDecodeWidth, new VmsIdAlloc)
    val vtypeWb = Input(Valid(new FuOutput(XLEN)))
    val redirect = Input(Valid(new Redirect))
    val flushMask = Output(UInt(size.W))
  })
  private val array = Reg(Vec(size, new VIWakeQueueEntry))

  io.flushMask := Cat(array.map(_.uop.robIdx.needFlush(io.redirect)).reverse)

  private val updateNetworkSeq = Seq.fill(size)(Module(new VIWakeQueueEntryUpdateNetwork))

  array.zip(updateNetworkSeq).zipWithIndex.foreach({case((a, un), idx) =>
    val enqSel = io.enq.map(e => e.wen && idx.U === e.addr)
    un.io.enq.valid := enqSel.reduce(_|_)
    un.io.enq.bits := Mux1H(enqSel, io.enq.map(_.data))
    un.io.entry := a
    un.io.robEnq := io.robEnq
    un.io.vmsResp.valid := io.vmsIdAllocte.en && io.vmsIdAllocte.addr === idx.U
    un.io.vmsResp.bits := io.vmsIdAllocte.data
    un.io.vtypeWb := io.vtypeWb
    when(un.io.updateEnable){
      a := un.io.entryNext
    }
  })

  private val readSel = array.indices.map(_.U === io.deq.addr)
  io.deq.data := Mux1H(readSel, array)

}