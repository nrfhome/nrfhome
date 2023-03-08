/*
 * Based on: https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <stdint.h>
#include "cc1101.h"

#define   WRITE_BURST       0x40            //write burst
#define   READ_SINGLE       0x80            //read single
#define   READ_BURST        0xC0            //read burst
#define   BYTES_IN_RXFIFO   0x7F            //byte number in RXfifo

typedef uint8_t byte;

static byte modulation = 2;
static byte frend0;
static byte chan = 0;
static int pa = 12;
static byte last_pa;
static bool ccmode = 0;
static float MHz = 433.92;
static byte m4RxBw = 0;
static byte m4DaRa __unused;
static byte m2DCOFF;
static byte m2MODFM;
static byte m2MANCH;
static byte m2SYNCM;
static byte m1FEC __unused;
static byte m1PRE __unused;
static byte m1CHSP __unused;
static byte pc1PQT __unused;
static byte pc1CRC_AF __unused;
static byte pc1APP_ST __unused;
static byte pc1ADRCHK __unused;
static byte pc0WDATA __unused;
static byte pc0PktForm __unused;
static byte pc0CRC_EN __unused;
static byte pc0LenConf __unused;
static byte trxstate = 0;
static byte clb1[2]= {24,28};
static byte clb2[2]= {31,38};
static byte clb3[2]= {65,76};
static byte clb4[2]= {77,79};

/****************************************************************/
uint8_t PA_TABLE[8]     = {0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00};
//                         -30  -20  -15  -10   0    5    7    10
uint8_t PA_TABLE_315[8] = {0x12,0x0D,0x1C,0x34,0x51,0x85,0xCB,0xC2,};             //300 - 348
uint8_t PA_TABLE_433[8] = {0x12,0x0E,0x1D,0x34,0x60,0x84,0xC8,0xC0,};             //387 - 464
//                          -30  -20  -15  -10  -6    0    5    7    10   12
uint8_t PA_TABLE_868[10] = {0x03,0x17,0x1D,0x26,0x37,0x50,0x86,0xCD,0xC5,0xC0,};  //779 - 899.99
//                          -30  -20  -15  -10  -6    0    5    7    10   11
uint8_t PA_TABLE_915[10] = {0x03,0x0E,0x1E,0x27,0x38,0x8E,0x84,0xCC,0xC3,0xC0,};  //900 - 928

LOG_MODULE_REGISTER(cc1101, LOG_LEVEL_INF);

#define SPI_DEV_NODE		DT_NODELABEL(spi0)
#define SPI_CS_NODE		DT_NODELABEL(spi_cs)

const struct device *spi_dev;

static struct spi_config spi_cfg = {
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		     SPI_TRANSFER_MSB,
	.frequency = 1000000,
};

static void SpiWriteReg(byte addr, byte value)
{
	byte rx_bytes[2] = {0xaa, 0xaa};
	byte tx_bytes[2] = {addr, value};
	struct spi_buf rx_buf = { rx_bytes, 2 };
	struct spi_buf tx_buf = { tx_bytes, 2 };
	struct spi_buf_set rx_buf_set = {&rx_buf, 1};
	struct spi_buf_set tx_buf_set = {&tx_buf, 1};

	int rc = spi_transceive(spi_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
	__ASSERT(rc == 0, "transceive failed");

	LOG_DBG("SPI> %02x %02x", tx_bytes[0], tx_bytes[1]);
}

static void SpiWriteBurstReg(byte addr, byte *buffer, byte num)
{
	byte tx_bytes[1] = {addr | WRITE_BURST};
	struct spi_buf tx_buf[2];
	struct spi_buf_set tx_buf_set = {tx_buf, 2};

	tx_buf[0].buf = tx_bytes;
	tx_buf[0].len = 1;
	tx_buf[1].buf = buffer;
	tx_buf[1].len = num;

	int rc = spi_transceive(spi_dev, &spi_cfg, &tx_buf_set, NULL);
	__ASSERT(rc == 0, "transceive failed");

	LOG_DBG("SPI> %02x ...", addr);
}

static byte SpiReadStatus(byte addr)
{
	byte rx_bytes[2] = {0xaa, 0xaa};
	byte tx_bytes[2] = {addr | READ_BURST, 0};
	struct spi_buf rx_buf = { rx_bytes, 2 };
	struct spi_buf tx_buf = { tx_bytes, 2 };
	struct spi_buf_set rx_buf_set = {&rx_buf, 1};
	struct spi_buf_set tx_buf_set = {&tx_buf, 1};

	int rc = spi_transceive(spi_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
	__ASSERT(rc == 0, "transceive failed");

	LOG_DBG("SPI> %02x", tx_bytes[0]);
	LOG_DBG("SPI< %02x %02x", rx_bytes[0], rx_bytes[1]);
	return rx_bytes[1];
}

static void SpiStrobe(byte cmd)
{
	byte rx_bytes[1] = {0xaa};
	byte tx_bytes[1] = {cmd};
	struct spi_buf rx_buf = { rx_bytes, 1 };
	struct spi_buf tx_buf = { tx_bytes, 1 };
	struct spi_buf_set rx_buf_set = {&rx_buf, 1};
	struct spi_buf_set tx_buf_set = {&tx_buf, 1};

	int rc = spi_transceive(spi_dev, &spi_cfg, &tx_buf_set, &rx_buf_set);
	__ASSERT(rc == 0, "transceive failed");

	LOG_DBG("SPI> %02x", tx_bytes[0]);
}

void setPA(int p)
{
int a = 0;
pa = p;

if (MHz >= 300 && MHz <= 348){
if (pa <= -30){a = PA_TABLE_315[0];}
else if (pa > -30 && pa <= -20){a = PA_TABLE_315[1];}
else if (pa > -20 && pa <= -15){a = PA_TABLE_315[2];}
else if (pa > -15 && pa <= -10){a = PA_TABLE_315[3];}
else if (pa > -10 && pa <= 0){a = PA_TABLE_315[4];}
else if (pa > 0 && pa <= 5){a = PA_TABLE_315[5];}
else if (pa > 5 && pa <= 7){a = PA_TABLE_315[6];}
else if (pa > 7){a = PA_TABLE_315[7];}
last_pa = 1;
}
else if (MHz >= 378 && MHz <= 464){
if (pa <= -30){a = PA_TABLE_433[0];}
else if (pa > -30 && pa <= -20){a = PA_TABLE_433[1];}
else if (pa > -20 && pa <= -15){a = PA_TABLE_433[2];}
else if (pa > -15 && pa <= -10){a = PA_TABLE_433[3];}
else if (pa > -10 && pa <= 0){a = PA_TABLE_433[4];}
else if (pa > 0 && pa <= 5){a = PA_TABLE_433[5];}
else if (pa > 5 && pa <= 7){a = PA_TABLE_433[6];}
else if (pa > 7){a = PA_TABLE_433[7];}
last_pa = 2;
}
else if (MHz >= 779 && MHz <= 899.99){
if (pa <= -30){a = PA_TABLE_868[0];}
else if (pa > -30 && pa <= -20){a = PA_TABLE_868[1];}
else if (pa > -20 && pa <= -15){a = PA_TABLE_868[2];}
else if (pa > -15 && pa <= -10){a = PA_TABLE_868[3];}
else if (pa > -10 && pa <= -6){a = PA_TABLE_868[4];}
else if (pa > -6 && pa <= 0){a = PA_TABLE_868[5];}
else if (pa > 0 && pa <= 5){a = PA_TABLE_868[6];}
else if (pa > 5 && pa <= 7){a = PA_TABLE_868[7];}
else if (pa > 7 && pa <= 10){a = PA_TABLE_868[8];}
else if (pa > 10){a = PA_TABLE_868[9];}
last_pa = 3;
}
else if (MHz >= 900 && MHz <= 928){
if (pa <= -30){a = PA_TABLE_915[0];}
else if (pa > -30 && pa <= -20){a = PA_TABLE_915[1];}
else if (pa > -20 && pa <= -15){a = PA_TABLE_915[2];}
else if (pa > -15 && pa <= -10){a = PA_TABLE_915[3];}
else if (pa > -10 && pa <= -6){a = PA_TABLE_915[4];}
else if (pa > -6 && pa <= 0){a = PA_TABLE_915[5];}
else if (pa > 0 && pa <= 5){a = PA_TABLE_915[6];}
else if (pa > 5 && pa <= 7){a = PA_TABLE_915[7];}
else if (pa > 7 && pa <= 10){a = PA_TABLE_915[8];}
else if (pa > 10){a = PA_TABLE_915[9];}
last_pa = 4;
}
if (modulation == 2){
PA_TABLE[0] = 0;  
PA_TABLE[1] = a;
}else{
PA_TABLE[0] = a;  
PA_TABLE[1] = 0; 
}
SpiWriteBurstReg(CC1101_PATABLE,PA_TABLE,8);
}

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Calibrate(void){

if (MHz >= 300 && MHz <= 348){
SpiWriteReg(CC1101_FSCTRL0, map(MHz, 300, 348, clb1[0], clb1[1]));
if (MHz < 322.88){SpiWriteReg(CC1101_TEST0,0x0B);}
else{
SpiWriteReg(CC1101_TEST0,0x09);
int s = SpiReadStatus(CC1101_FSCAL2);
if (s<32){SpiWriteReg(CC1101_FSCAL2, s+32);}
if (last_pa != 1){setPA(pa);}
}
}
else if (MHz >= 378 && MHz <= 464){
SpiWriteReg(CC1101_FSCTRL0, map(MHz, 378, 464, clb2[0], clb2[1]));
if (MHz < 430.5){SpiWriteReg(CC1101_TEST0,0x0B);}
else{
SpiWriteReg(CC1101_TEST0,0x09);
int s = SpiReadStatus(CC1101_FSCAL2);
if (s<32){SpiWriteReg(CC1101_FSCAL2, s+32);}
if (last_pa != 2){setPA(pa);}
}
}
else if (MHz >= 779 && MHz <= 899.99){
SpiWriteReg(CC1101_FSCTRL0, map(MHz, 779, 899, clb3[0], clb3[1]));
if (MHz < 861){SpiWriteReg(CC1101_TEST0,0x0B);}
else{
SpiWriteReg(CC1101_TEST0,0x09);
int s = SpiReadStatus(CC1101_FSCAL2);
if (s<32){SpiWriteReg(CC1101_FSCAL2, s+32);}
if (last_pa != 3){setPA(pa);}
}
}
else if (MHz >= 900 && MHz <= 928){
SpiWriteReg(CC1101_FSCTRL0, map(MHz, 900, 928, clb4[0], clb4[1]));
SpiWriteReg(CC1101_TEST0,0x09);
int s = SpiReadStatus(CC1101_FSCAL2);
if (s<32){SpiWriteReg(CC1101_FSCAL2, s+32);}
if (last_pa != 4){setPA(pa);}
}
}

void setMHZ(float mhz){
byte freq2 = 0;
byte freq1 = 0;
byte freq0 = 0;

MHz = mhz;

for (bool i = 0; i==0;){
if (mhz >= 26){
mhz-=26;
freq2+=1;
}
else if (mhz >= 0.1015625){
mhz-=0.1015625;
freq1+=1;
}
else if (mhz >= 0.00039675){
mhz-=0.00039675;
freq0+=1;
}
else{i=1;}
}
if (freq0 > 255){freq1+=1;freq0-=256;}

SpiWriteReg(CC1101_FREQ2, freq2);
SpiWriteReg(CC1101_FREQ1, freq1);
SpiWriteReg(CC1101_FREQ0, freq0);

Calibrate();
}

void Split_MDMCFG2(void){
int calc = SpiReadStatus(18);
m2DCOFF = 0;
m2MODFM = 0;
m2MANCH = 0;
m2SYNCM = 0;
for (bool i = 0; i==0;){
if (calc >= 128){calc-=128; m2DCOFF+=128;}
else if (calc >= 16){calc-=16; m2MODFM+=16;}
else if (calc >= 8){calc-=8; m2MANCH+=8;}
else{m2SYNCM = calc; i=1;}
}
}

void setModulation(byte m){
if (m>4){m=4;}
modulation = m;
Split_MDMCFG2();
switch (m)
{
case 0: m2MODFM=0x00; frend0=0x10; break; // 2-FSK
case 1: m2MODFM=0x10; frend0=0x10; break; // GFSK
case 2: m2MODFM=0x30; frend0=0x11; break; // ASK
case 3: m2MODFM=0x40; frend0=0x10; break; // 4-FSK
case 4: m2MODFM=0x70; frend0=0x10; break; // MSK
}
SpiWriteReg(CC1101_MDMCFG2, m2DCOFF+m2MODFM+m2MANCH+m2SYNCM);
SpiWriteReg(CC1101_FREND0,   frend0);
setPA(pa);
}

void setCCMode(bool s){
ccmode = s;
if (ccmode == 1){
SpiWriteReg(CC1101_IOCFG2,      0x0B);
SpiWriteReg(CC1101_IOCFG0,      0x06);
SpiWriteReg(CC1101_PKTCTRL0,    0x05);
SpiWriteReg(CC1101_MDMCFG3,     0xF8);
SpiWriteReg(CC1101_MDMCFG4,11+m4RxBw);
}else{
SpiWriteReg(CC1101_IOCFG2,      0x0D);
SpiWriteReg(CC1101_IOCFG0,      0x0D);
SpiWriteReg(CC1101_PKTCTRL0,    0x32);
SpiWriteReg(CC1101_MDMCFG3,     0x93);
SpiWriteReg(CC1101_MDMCFG4, 7+m4RxBw);
}
setModulation(modulation);
}

void RegConfigSettings(void) 
{   
    SpiWriteReg(CC1101_FSCTRL1,  0x06);

    setCCMode(ccmode);
    setMHZ(MHz);

    SpiWriteReg(CC1101_MDMCFG1,  0x02);
    SpiWriteReg(CC1101_MDMCFG0,  0xF8);
    SpiWriteReg(CC1101_CHANNR,   chan);
    SpiWriteReg(CC1101_DEVIATN,  0x47);
    SpiWriteReg(CC1101_FREND1,   0x56);
    SpiWriteReg(CC1101_MCSM0 ,   0x18);
    SpiWriteReg(CC1101_FOCCFG,   0x16);
    SpiWriteReg(CC1101_BSCFG,    0x1C);
    SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
    SpiWriteReg(CC1101_AGCCTRL1, 0x00);
    SpiWriteReg(CC1101_AGCCTRL0, 0xB2);
    SpiWriteReg(CC1101_FSCAL3,   0xE9);
    SpiWriteReg(CC1101_FSCAL2,   0x2A);
    SpiWriteReg(CC1101_FSCAL1,   0x00);
    SpiWriteReg(CC1101_FSCAL0,   0x1F);
    SpiWriteReg(CC1101_FSTEST,   0x59);
    SpiWriteReg(CC1101_TEST2,    0x81);
    SpiWriteReg(CC1101_TEST1,    0x35);
    SpiWriteReg(CC1101_TEST0,    0x09);
    SpiWriteReg(CC1101_PKTCTRL1, 0x04);
    SpiWriteReg(CC1101_ADDR,     0x00);
    SpiWriteReg(CC1101_PKTLEN,   0x00);
}

void SetTx(float mhz)
{
  SpiStrobe(CC1101_SIDLE);
  setMHZ(mhz);
  SpiStrobe(CC1101_STX);        //start send
  trxstate=1;
}

void setSidle(void)
{
  SpiStrobe(CC1101_SIDLE);
  trxstate=0;
}

void cc1101_init(void)
{
#if DT_NODE_EXISTS(SPI_CS_NODE) && DT_NODE_EXISTS(SPI_DEV_NODE)
	static const struct spi_cs_control spi_dev_cs_ctrl = {
		.gpio = GPIO_DT_SPEC_GET(SPI_CS_NODE, gpios),
		.delay = 1,
	};
	spi_cfg.cs = &spi_dev_cs_ctrl;

	spi_dev = DEVICE_DT_GET(SPI_DEV_NODE);
	__ASSERT(device_is_ready(spi_dev), "SPI master unavailable");

	gpio_pin_configure_dt(&spi_dev_cs_ctrl.gpio, GPIO_OUTPUT_INACTIVE);

	// Reset
	gpio_pin_set_dt(&spi_dev_cs_ctrl.gpio, 0);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&spi_dev_cs_ctrl.gpio, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&spi_dev_cs_ctrl.gpio, 0);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&spi_dev_cs_ctrl.gpio, 1);
	k_sleep(K_MSEC(1));

	SpiStrobe(CC1101_SRES);
	k_sleep(K_MSEC(10));

	RegConfigSettings();
#endif
}

void cc1101_set_freq(float freq)
{
	if (spi_dev) {
		SetTx(freq);
	}
}
