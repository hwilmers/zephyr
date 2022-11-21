/*
 * Copyright (c) 2020 Endian Technologies AB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GSM_PPP_H_
#define GSM_PPP_H_

#define GSM_MODEM_DEVICE_NAME "modem_gsm"

/** @cond INTERNAL_HIDDEN */
struct device;
void gsm_ppp_start(const struct device *dev);
void gsm_ppp_stop(const struct device *dev);
/** @endcond */

#if defined(CONFIG_MODEM_GSM_CONFIG)
struct modem_gsm_config {
	const char *uart_name;
	const char *apn;
#if defined(CONFIG_GSM_MUX)
	bool use_mux;
#endif
};
extern struct modem_gsm_config gsm_config;
#endif

#endif /* GSM_PPP_H_ */
