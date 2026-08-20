/* SPDX-License-Identifier: GPL-2.0-or-later */
/* RTW8852C, rfe_type 53 with an external front-end module: the receive-gain
 * program the vendor's own firmware loads for this part.
 *
 * The table itself is GENERATED -- see rtw8852c_rfe53_gain.c and
 * dev/re-tools/rfe53_gain_8852c.py.
 */

#ifndef __RTW89_8852C_RFE53_GAIN_H__
#define __RTW89_8852C_RFE53_GAIN_H__

#include "core.h"

/* The rfe_type this program belongs to.  It is the ONLY discriminator rtw89
 * offers: the efuse reports an rfe_type, and there is no FEM-package field, so
 * a board that is rfe_type 53 with a DIFFERENT front-end module would take
 * these constants wrongly.  That limit is real and is why the apply path is
 * gated on the value rather than applied unconditionally.
 */
#define RTW8852C_RFE_TYPE_EFEM	53

extern const struct rtw89_reg2_def rtw8852c_rfe53_fem_gain[];
extern const unsigned int rtw8852c_rfe53_fem_gain_num;

void rtw8852c_apply_rfe53_fem_gain(struct rtw89_dev *rtwdev);

#endif
