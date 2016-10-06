/*
 * collectd - src/sff_dmi.c
 * Copyright 2015 Cumulus Networks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "collectd.h"
#include "common.h"
#include "plugin.h"

#if HAVE_SYS_IOCTL_H
# include <sys/ioctl.h>
#endif
#if HAVE_NET_IF_H
# include <net/if.h>
#endif
#if HAVE_LINUX_SOCKIOS_H
# include <linux/sockios.h>
#endif
#if HAVE_LINUX_ETHTOOL_H
# include <linux/ethtool.h>
#endif
#include <arpa/inet.h>
#include <sys/types.h>
#include <ifaddrs.h>

#define SFF_A0_BASE 0x0
#define SFF_A2_BASE 0x100

#define SFF_ID_ADDR (SFF_A0_BASE + 0x0)
#define SFF_ID_LEN  1
#define SFF_ID_SFP  0x3

#define SFF_DMI_TYPE_ADDR           (SFF_A0_BASE + 92)
#define SFF_DMI_TYPE_LEN            1
#define SFF_DMI_TYPE_HAS_DMI_MASK   (1 << 6)
#define SFF_DMI_TYPE_INT_CALIB_MASK (1 << 5)
#define SFF_DMI_TYPE_POWER_AVG_MASK (1 << 3)

#define SFF_DMI_DATA_ADDR (SFF_A2_BASE + 96)
#define SFF_DMI_DATA_LEN  10

struct sff_dmi_sample {
  gauge_t temp;
  gauge_t vcc;
  gauge_t tx_bias;
  gauge_t tx_power;
  gauge_t rx_power;
};

static void
sff_dmi_submit_sample(const char *ifname, struct sff_dmi_sample *sample)
{
  value_list_t vl = VALUE_LIST_INIT;
  value_t value;

  vl.values = &value;
  vl.values_len = 1;

  sstrncpy(vl.host, hostname_g, sizeof(vl.host));
  sstrncpy(vl.plugin, "sff_dmi", sizeof(vl.plugin));
  sstrncpy(vl.plugin_instance, ifname, sizeof(vl.plugin_instance));

  sstrncpy(vl.type, "temperature", sizeof(vl.type));
  sstrncpy(vl.type_instance, "transceiver", sizeof(vl.type_instance));
  value.gauge = sample->temp;
  plugin_dispatch_values(&vl);

  sstrncpy(vl.type, "voltage", sizeof(vl.type));
  sstrncpy(vl.type_instance, "supply", sizeof(vl.type_instance));
  value.gauge = sample->vcc;
  plugin_dispatch_values(&vl);

  sstrncpy(vl.type, "current", sizeof(vl.type));
  sstrncpy(vl.type_instance, "tx-bias", sizeof(vl.type_instance));
  value.gauge = sample->tx_bias;
  plugin_dispatch_values(&vl);

  sstrncpy(vl.type, "power", sizeof(vl.type));
  sstrncpy(vl.type_instance, "tx", sizeof(vl.type_instance));
  value.gauge = sample->tx_power;
  plugin_dispatch_values(&vl);

  sstrncpy(vl.type, "power", sizeof(vl.type));
  sstrncpy(vl.type_instance, "rx", sizeof(vl.type_instance));
  value.gauge = sample->rx_power;
  plugin_dispatch_values(&vl);
}

static void
sff_dmi_convert(uint8_t *data, struct sff_dmi_sample *sample)
{
  // 16-bit 2's compliment 1/256 degC
  sample->temp     = (gauge_t)(ntohs(*(uint16_t *)&data[0]) / 256.0);
  // 16-bit unsigned 1uV
  sample->vcc      = (gauge_t)(ntohs(*(uint16_t *)&data[2]) / 10000.0);
  // 16-bit unsigned 2uA
  sample->tx_bias  = (gauge_t)(ntohs(*(uint16_t *)&data[4]) / 500000.0);
  // 16-bit unsigned .1uW
  sample->tx_power = (gauge_t)(ntohs(*(uint16_t *)&data[6]) / 1000000.0);
  // 16-bit unsigned .1uW
  sample->rx_power = (gauge_t)(ntohs(*(uint16_t *)&data[8]) / 1000000.0);
  ERROR("sample %u,%u:%u,%u:%u,%u:%u,%u:%u,%u -> %f:%f:%f:%f:%f\n",
        data[0], data[1],
        data[2], data[3],
        data[4], data[5],
        data[6], data[7],
        data[8], data[9],
        sample->temp,
        sample->vcc,
        sample->tx_bias,
        sample->tx_power,
        sample->rx_power);
}

static int
sff_dmi_read_intf(int fd, const char *ifname)
{
  struct {
    struct ethtool_eeprom eepromreq;
    union {
      uint8_t id;
      uint8_t dmi_type;
      uint8_t dmi_data[10];
      uint8_t buf[256];
    };
  } eeprom = { .eepromreq.cmd = ETHTOOL_GMODULEEEPROM };
  struct ifreq ifreq = { .ifr_data = (char *)&eeprom };
  struct sff_dmi_sample sample;
  int err;

  sstrncpy(ifreq.ifr_name, ifname, sizeof(ifreq.ifr_name));

  eeprom.eepromreq.offset = SFF_ID_ADDR;
  eeprom.eepromreq.len = SFF_ID_LEN;
  err = ioctl(fd, SIOCETHTOOL, &ifreq);
  if (err) {
    ERROR("sff_dmi: %s ifreq SFF_ID failed: 0x%x\n", ifname, err);
    return err;
  }

  if (eeprom.id != SFF_ID_SFP) {
    // XXX - only SFP implemented
    ERROR("sff_dmi: %s is not SFP\n", ifname);
    return -1;
  }

  eeprom.eepromreq.offset = SFF_DMI_TYPE_ADDR;
  eeprom.eepromreq.len = SFF_DMI_TYPE_LEN;
  err = ioctl(fd, SIOCETHTOOL, &ifreq);
  if (err) {
    ERROR("sff_dmi: %s ifreq SFF_DMI_TYPE failed: 0x%x\n", ifname, err);
    return err;
  }

  ERROR("sff_dmi: %s dmi type: 0x%x\n", ifname, eeprom.dmi_type);
  if (!(eeprom.dmi_type & SFF_DMI_TYPE_HAS_DMI_MASK)) {
    ERROR("sff_dmi: %s no DMI\n", ifname);
    return -1;
  }

  if (!(eeprom.dmi_type & SFF_DMI_TYPE_POWER_AVG_MASK)) {
    // XXX - OMA not implemented
    ERROR("sff_dmi: %s is OMA\n", ifname);
    return -1;
  }

  if (!(eeprom.dmi_type & SFF_DMI_TYPE_INT_CALIB_MASK)) {
    // XXX - uncalibrated sensors not implemented
    ERROR("sff_dmi: %s is uncalibrated\n", ifname);
    return -1;
  }

  eeprom.eepromreq.offset = SFF_DMI_DATA_ADDR;
  eeprom.eepromreq.len = SFF_DMI_DATA_LEN;
  err = ioctl(fd, SIOCETHTOOL, &ifreq);
  if (err) {
    ERROR("sff_dmi: %s ifreq SFF_DMI_DATA failed: 0x%x\n", ifname, err);
    return err;
  }

  ERROR("sff_dmi: %s converting\n", ifname);
  sff_dmi_convert(&eeprom.dmi_data[0], &sample);
  sff_dmi_submit_sample(ifname, &sample);

  return 0;
}

static int sff_dmi_read(void)
{
  struct ifaddrs *if_list;
  struct ifaddrs *if_ptr;
  int fd;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    char msg[1024];
    ERROR("sff_dmi plugin: failed to open socket: %s",
          sstrerror(errno, msg, sizeof(msg)));
    return 1;
  }

  if (getifaddrs(&if_list) != 0)
    return 1;

  for (if_ptr = if_list; if_ptr != NULL; if_ptr = if_ptr->ifa_next) {
    sff_dmi_read_intf(fd, if_ptr->ifa_name);
  }

  freeifaddrs(if_list);
  close(fd);
  return 0;
}

static int sff_dmi_shutdown (void)
{
  return 0;
}

void module_register (void)
{
  plugin_register_read ("sff_dmi", sff_dmi_read);
  plugin_register_shutdown ("sff_dmi", sff_dmi_shutdown);
}
