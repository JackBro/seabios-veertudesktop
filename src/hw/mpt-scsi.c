// (qemu-emulated) Fusion MPT boot support.
//
// Copyright (C) 2012 Red Hat Inc.
//
// Authors:
//  Gerd Hoffmann <kraxel at redhat.com>
//
// based on virtio-scsi.c which is written by:
//  Paolo Bonzini <pbonzini at redhat.com>
//
// This file may be distributed under the terms of the GNU LGPLv3 license.

#include "biosvar.h" // GET_GLOBALFLAT
#include "block.h" // struct drive_s
#include "blockcmd.h" // scsi_drive_setup
#include "config.h" // CONFIG_*
#include "fw/paravirt.h" // runningOnQEMU
#include "malloc.h" // free
#include "output.h" // dprintf
#include "pci.h" // foreachpci
#include "pci_ids.h" // PCI_DEVICE_ID
#include "pci_regs.h" // PCI_VENDOR_ID
#include "std/disk.h" // DISK_RET_SUCCESS
#include "string.h" // memset
#include "util.h" // usleep

#define MPT_REG_DOORBELL  0x00
#define MPT_REG_WRITE_SEQ 0x04
#define MPT_REG_HOST_DIAG 0x08
#define MPT_REG_TEST      0x0c
#define MPT_REG_DIAG_DATA 0x10
#define MPT_REG_DIAG_ADDR 0x14
#define MPT_REG_ISTATUS   0x30
#define MPT_REG_IMASK     0x34
#define MPT_REG_REQ_Q     0x40
#define MPT_REG_REP_Q     0x44

#define MPT_DOORBELL_MSG_RESET 0x40
#define MPT_DOORBELL_HANDSHAKE 0x42
#define MPT_DOORBELL_REG_FUNC(f, s) ((((f)&0xff)<<24)|(((s)&0xff)<<16))

#define MPT_IMASK_DOORBELL 0x01
#define MPT_IMASK_REPLY    0x08

struct mpt_lun_s {
    struct drive_s drive;
    struct pci_device *pci;
    u32 iobase;
    u8 target;
    u8 lun;
};

__attribute__((aligned(8))) u8 reply_msg[128] VARFSEG;


#define MPT_MESSAGE_HDR_FUNCTION_SCSI_IO_REQUEST        (0x00)
#define MPT_MESSAGE_HDR_FUNCTION_IOC_INIT               (0x02)

static struct MptIOCInitRequest
{
    u8     WhoInit;             /* Which system sent this init request. */
    u8     Reserved1;           /* Reserved */
    u8     ChainOffset;         /* Chain offset in the SG list. */
    u8     Function;            /* Function to execute. */
    u8     Flags;               /* Flags */
    u8     MaxDevices;          /* Max devices the driver can handle. */
    u8     MaxBuses;            /* Max buses the driver can handle. */
    u8     MessageFlags;        /* Message flags. */
    u32    MessageContext;      /* Message context ID. */
    u16    ReplyFrameSize;      /* Reply frame size. */
    u16    Reserved2;           /* Reserved */
    u32    HostMfaHighAddr;     /* Upper 32bit of the message frames. */
    u32    SenseBufferHighAddr; /* Upper 32bit of the sense buffer. */
} __attribute__((aligned(8))) MptIOCInitRequest = {
        2,
        0,
        0,
        MPT_MESSAGE_HDR_FUNCTION_IOC_INIT,
        0,
        8,
        1,
        0,
        0,
        sizeof(reply_msg),
        0,
        0,
        0
};

struct MptIOCInitReply {
    u8     WhoInit;     /* Which subsystem sent this init request. */
    u8     Reserved1;   /* Reserved */
    u8     MessageLength; /* Message length */
    u8     Function;    /* Function. */
    u8     Flags;       /* Flags */
    u8     MaxDevices;  /* Maximum number of devices the driver can handle. */
    u8     MaxBuses;    /* Maximum number of busses the driver can handle. */
    u8     MessageFlags; /* Message flags. */
    u32    MessageContext; /* Message context ID */
    u16    Reserved2;   /* Reserved */
    u16    IOCStatus;   /* IO controller status. */
    u32    IOCLogInfo;  /* IO controller log information. */
};

#pragma pack(1)
typedef struct MptSCSIIORequest {
    u8     TargetID;            /* Target ID */
    u8     Bus;                 /* Bus number */
    u8     ChainOffset;         /* Chain offset */
    u8     Function;            /* Function number. */
    u8     CDBLength;           /* CDB length. */
    u8     SenseBufferLength;   /* Sense buffer length. */
    u8     Reserved;            /* Reserved */
    u8     MessageFlags;        /* Message flags. */
    u32    MessageContext;      /* Message context ID. */
    u8     LUN[8];              /* LUN */
    u32    Control;             /* Control values. */
    u8     CDB[16];             /* The CDB. */
    u32    DataLength;          /* Data length. */
    u32    SenseBufferLowAddr;  /* Sense buffer low 32bit address. */
} MptSCSIIORequest_t;

#define MPT_CONTEXT_MAGIC 0xaaaa5555

typedef struct MptSGEntrySimple32 {
    unsigned u24Length:24;
    unsigned fEndOfList:1;
    unsigned f64BitAddress:1;
    unsigned fBufferContainsData:1;
    unsigned fLocalAddress:1;
    unsigned u2ElementType:2;
    unsigned fEndOfBuffer:1;
    unsigned fLastElement:1;
    u32 DataBufferAddressLow;
} MptSGEntrySimple32_t;
#pragma pack(0)

static int
mpt_scsi_cmd(struct mpt_lun_s *llun, struct disk_op_s *op,
             void *cdbcmd, u16 target, u16 lun, u16 blocksize)
{

    u32 iobase = GET_GLOBALFLAT(llun->iobase);
    int i;
    u8 sense_buf[32];
    int retry = 0;

    struct __attribute__ ((__packed__)) scsi_req {
        MptSCSIIORequest_t      scsi_io;
        MptSGEntrySimple32_t    sge;
    } __attribute__((aligned(8))) req = {
        {
        target,
        0,
        0,
        MPT_MESSAGE_HDR_FUNCTION_SCSI_IO_REQUEST,
        16,
        32,
        0,
        0,
        MPT_CONTEXT_MAGIC,
        {0, lun, 0, 0, 0, 0, 0, 0},
        0,
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        (op->count * blocksize),
        (u32)MAKE_FLATPTR(GET_SEG(SS), &sense_buf[0])
        },
        {
        op->count * blocksize,
        1,
        0,
        0,
        0,
        1,
        1,
        1,
        (u32)op->buf_fl
        }
    };

    if (lun != 0) {
        return DISK_RET_ENOTREADY;
    }

    if (blocksize == 0) {
        req.scsi_io.Control = 0;
    } else if (cdb_is_read(cdbcmd, blocksize)) {
        req.scsi_io.Control = 2 << 24;
    } else {
        req.scsi_io.Control = 1 << 24;
        req.sge.fBufferContainsData = 1;
    }

    u8 *cdb = (u8 *)cdbcmd;
    for (i = 0; i < 16; i++) {
        req.scsi_io.CDB[i] = cdb[i];
    }

dprintf(1, "op->buf_fl %x\n",  (u32)op->buf_fl);

try_again:
    outl((u32)MAKE_FLATPTR(GET_SEG(SS), &req), iobase + MPT_REG_REQ_Q);

    for (;;) {
        u32 istatus = inl(iobase + MPT_REG_ISTATUS);
        u32 resp;
        if (istatus & MPT_IMASK_REPLY) {
            do {
                resp = inl(iobase + MPT_REG_REP_Q);
                if (resp == MPT_CONTEXT_MAGIC) {
                    return DISK_RET_SUCCESS;
                } else if ((resp << 1) == (u32)&reply_msg[0] || (resp << 1) == 0) {
                    if (retry == 0) {
                        retry = 1;
                        goto try_again;
                    }
                    return DISK_RET_EBADTRACK;
                }
            } while (resp != 0xffffffff);
        }
        usleep(50);
    }
}

int
mpt_scsi_cmd_data(struct disk_op_s *op, void *cdbcmd, u16 blocksize)
{
    if (!CONFIG_MPT_SCSI) {
        return DISK_RET_EBADTRACK;
    }

    struct mpt_lun_s *llun =
        container_of(op->drive_gf, struct mpt_lun_s, drive);

    return mpt_scsi_cmd(llun, op, cdbcmd,
                        GET_GLOBALFLAT(llun->target), GET_GLOBALFLAT(llun->lun),
                        blocksize);
}

static int
mpt_scsi_add_lun(struct pci_device *pci, u32 iobase, u8 target, u8 lun)
{
    struct mpt_lun_s *llun = malloc_fseg(sizeof(*llun));
    if (!llun) {
        warn_noalloc();
        return -1;
    }
    memset(llun, 0, sizeof(*llun));
    llun->drive.type = DTYPE_MPT_SCSI;
    llun->drive.cntl_id = pci->bdf;
    llun->pci = pci;
    llun->target = target;
    llun->lun = lun;
    llun->iobase = iobase;

    char *name = znprintf(16, "mpt %02x:%02x.%x %d:%d",
                          pci_bdf_to_bus(pci->bdf), pci_bdf_to_dev(pci->bdf),
                          pci_bdf_to_fn(pci->bdf), target, lun);
    int prio = bootprio_find_scsi_device(pci, target, lun);
    int ret = scsi_drive_setup(&llun->drive, name, prio);
    free(name);
    if (ret) {
        goto fail;
    }
    return 0;

fail:
    free(llun);
    return -1;
}

static void
mpt_scsi_scan_target(struct pci_device *pci, u32 iobase, u8 target)
{
    /* TODO: send REPORT LUNS.  For now, only LUN 0 is recognized.  */
    mpt_scsi_add_lun(pci, iobase, target, 0);
}

static void
init_mpt_scsi(struct pci_device *pci, const char *dev_name)
{
    u32 *msg_out_p;
    u16 *msg_in_p;
    u16 bdf = pci->bdf;
    u32 iobase = pci_config_readl(pci->bdf, PCI_BASE_ADDRESS_0)
        & PCI_BASE_ADDRESS_IO_MASK;
    struct MptIOCInitReply MptIOCInitReply;

    dprintf(1, "found %s at %02x:%02x.%x, io @ %x\n", dev_name,
            pci_bdf_to_bus(bdf), pci_bdf_to_dev(bdf),
            pci_bdf_to_fn(bdf), iobase);

    // reset
    outl(MPT_DOORBELL_REG_FUNC(MPT_DOORBELL_MSG_RESET, 0),
        iobase + MPT_REG_DOORBELL);
    outl(MPT_IMASK_DOORBELL|MPT_IMASK_REPLY , iobase + MPT_REG_IMASK);
    outl(0, iobase + MPT_REG_ISTATUS);

    outl(MPT_DOORBELL_REG_FUNC(MPT_DOORBELL_HANDSHAKE,
        sizeof(MptIOCInitRequest)/sizeof(u32)), iobase + MPT_REG_DOORBELL);
    msg_out_p = (u32 *)&MptIOCInitRequest;
    int i;
    for (i = 0; i < sizeof(MptIOCInitRequest)/sizeof(u32); i++) {
        outl(*msg_out_p++, iobase + MPT_REG_DOORBELL);
    }

    msg_in_p = (u16 *)&MptIOCInitReply;
    for (i = 0; i < sizeof(MptIOCInitReply)/sizeof(u16); i++)
        /* Lower 16 bits has the message */
        *msg_in_p++ = (u16)inl(iobase + MPT_REG_DOORBELL);
    /* TODO: Should make sure reply message looks good */

    outl(0, iobase + MPT_REG_ISTATUS);

    //outl((u32)MAKE_FLATPTR(GET_SEG(SS), &reply_msg[0]), iobase + MPT_REG_REP_Q);
    outl((u32)&reply_msg[0], iobase + MPT_REG_REP_Q);

    for (i = 0; i < 7; i++) {
        mpt_scsi_scan_target(pci, iobase, i);
    }

    return;
}

void
mpt_scsi_setup(void)
{
    ASSERT32FLAT();
    if (!CONFIG_MPT_SCSI) {
        return;
    }

    memset(reply_msg, 0, sizeof(reply_msg));
    dprintf(3, "init MPT\n");

    struct pci_device *pci;
    foreachpci(pci) {
        if (pci->vendor == PCI_VENDOR_ID_LSI_LOGIC) {
            if (pci->device == PCI_DEVICE_ID_LSI_53C1030) {
                init_mpt_scsi(pci, "lsi53c1030");
            }
            if (pci->device == PCI_DEVICE_ID_LSI_SAS1068) {
                init_mpt_scsi(pci, "sas1068");
            }
            if (pci->device == PCI_DEVICE_ID_LSI_SAS1068E) {
                init_mpt_scsi(pci, "sas1068e");
            }
        }
    }
}
