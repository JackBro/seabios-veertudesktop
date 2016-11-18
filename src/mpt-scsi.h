#ifndef __MPT_SCSI_H
#define __MPT_SCSI_H

struct disk_op_s;
int mpt_scsi_cmd_data(struct disk_op_s *op, void *cdbcmd, u16 blocksize);
void mpt_scsi_setup(void);

#endif /* __MPT_SCSI_H */
