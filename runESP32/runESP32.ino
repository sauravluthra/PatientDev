// Talks to multiple BNO085 sensors via SPI.
// This is experimental demonstration code, so you may discover problems.
// I've run this with up to three BNO devices (that's all I have).
//
// Uses NED/aerospace coordinates, so my axes definitions may differ from your other projects.
// My BNO08x orientation dot is towards left rear, so code flips various sensor axes and quaternions accordingly.
// Outputs data in compact Base64 format.
//
// To program Adafruit Feather M4 Express 3857:  select "Adafruit Feather M4 Express (SAMD51)", COMxx, "AVRISP mkII". Beware COM port may annoyingly change itself.
//
// Connect Feather "MI"  to "SDA" of all BNOs
// Connect Feather "MO"  to "DI"  of all BNOs
// Connect Feather "SCK" to "SCL" of all BNOs
//
// Bugs:
// Occasionally, a random BNO seems to not receive one of the report requests, causing it to not output that report, but the problem has mysteriously vanished.

#include <SPI.h>

#define BNOs        7                        // number of BNO08x breakouts (you must also comment-out unused "isr" definitions, see below)
#define pinRST      4//20                        // connect this  Feather pin  to "RST" of all BNOs
#define pinsINT     22,27,32,25,34,36,35//29, 10,  6, 8,  4,  2,  5       // connect these Feather pins to "INT" of BNO0,BNO1,BNO2
#define pinsCS      16,26,13,33,21,1,2//21,  9, 13, 7, 26, 28, 19       // connect these Feather pins to "CS"  of BNO0,BNO1,BNO2

#define ALL_REPORTS 1           // 0 request only quaternions, 1 request all reports
#define REPORT_TIME 2500L       // time between sensor reports, microseconds  (2500L 400 Hz, 10000L 100 Hz, 100000L 10 Hz)
#define SPI_CLOCK   3000000L    // SPI clock rate, hertz
#define SERIAL_BAUD 230400L     // serial port baud rate, hertz

#define DATAOUT     0           // 1 enables data output messages
#define STATS       1           // 1 enables report counting statistics
#define DEBUG       0           // 1 enables verbose (overwhelming) info messages, suggest also greatly increasing REPORT_TIME

#define SPI_SETTINGS SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE3)

#define numberof(x)  (sizeof(x) / sizeof(*(x)))

// *******************
// **  Output data  **
// *******************

int16_t iax[BNOs], iay[BNOs], iaz[BNOs];             // accel, integer
int16_t igx[BNOs], igy[BNOs], igz[BNOs];             // gyro, integer
int16_t imx[BNOs], imy[BNOs], imz[BNOs];             // magneto, integer
int16_t ilx[BNOs], ily[BNOs], ilz[BNOs];             // linear accel, integer
int16_t iqw[BNOs], iqx[BNOs], iqy[BNOs], iqz[BNOs];  // quaternion, integer

char obuf[70], *pbuf;               // ensure this output buffer is big enough for your output string!

static void uart_b64(int32_t i)     // output 18-bit integer as compact 3-digit base64
{
  for (int n=12; n >= 0; n-=6)
  {
    uint8_t c = (i >> n) & 63;
    *pbuf++ = (char)(c<26 ? 'A'+c : c<52 ? 'a'-26+c : c<62 ? '0'-52+c : c==62 ? '+' : '/');
  }
}

#if ALL_REPORTS
  static void output_data(uint8_t bno)
  {
    float kACC = 1/9.80665/256 * 131072/10.0;   // scale units for my project
    float kGYR =  180/M_PI/512 * 131072/4000.0;
    float kMAG =       0.01/16 * 131072/1.0;
    float kLAC = 1/9.80665/256 * 131072/10.0;

    pbuf = obuf;                        // pointer into output buffer
    *pbuf++ = 'k';  *pbuf++ = 'q'+bno;  // string header "kq" is BNO0, "kr" is BNO1, "ks" is BNO2, etc
    uart_b64(kACC*iax[bno]);  uart_b64(-kACC*iay[bno]);  uart_b64(-kACC*iaz[bno]);  // accel,   convert from m/sec/sec*256 to       g*131072/10.0
    uart_b64(kGYR*igx[bno]);  uart_b64(-kGYR*igy[bno]);  uart_b64(-kGYR*igz[bno]);  // gyro,    convert from   rad/sec*512 to deg/sec*131072/4000.0
    uart_b64(kMAG*imx[bno]);  uart_b64(-kMAG*imy[bno]);  uart_b64(-kMAG*imz[bno]);  // magneto, convert from        uT*16  to   gauss*131072/1.0
    uart_b64(kLAC*ilx[bno]);  uart_b64(-kLAC*ily[bno]);  uart_b64(-kLAC*ilz[bno]);  // linacc,  convert from m/sec/sec*256 to       g*131072/10.0
    uart_b64(iqw[bno]+iqz[bno]); uart_b64(iqx[bno]+iqy[bno]); uart_b64(iqx[bno]-iqy[bno]); uart_b64(iqw[bno]-iqz[bno]);  // quat,    rotate quat to my reference frame, do no conversion because it'll be normalized later
    uart_b64(0);            // temp,    convert from    degC*128     to  degC*131072/100.0
    uart_b64(0);            // baro,    convert from hectoPa*1048576 to  mbar*131072/2000.0
    uart_b64(0xFF);         // status,  four 2-bit codes {sys,gyr,acc,mag}
    *pbuf++ = 13;           // CR LF
    *pbuf++ = 10;
    *pbuf++ = 0;            // terminate string
    Serial.write(obuf);     // writing one long string is *much* faster than printing individual values
  }
#else
  static void output_data(uint8_t bno)
  {
    pbuf = obuf;                        // pointer into output buffer
    *pbuf++ = 'k';  *pbuf++ = 'q'+bno;  // string header "kq" is BNO0, "kr" is BNO1, "ks" is BNO2, etc
    // my BNO08x orientation dot is towards left rear, rotate BNO08x quaternion to NED conventions
    uart_b64(iqw[bno]+iqz[bno]); uart_b64(iqx[bno]+iqy[bno]); uart_b64(iqx[bno]-iqy[bno]); uart_b64(iqw[bno]-iqz[bno]);  // quat,    rotate quat to my reference frame, do no conversion because it'll be normalized later
    *pbuf++ = 13;           // CR LF
    *pbuf++ = 10;
    *pbuf++ = 0;            // terminate string
    Serial.write(obuf);     // writing one long string is *much* faster than printing individual values
  }
#endif

// ******************************************************************************************************
// **  Receive one SPI byte while simultaneously sending one byte from a queue of pending tx messages  **
// ******************************************************************************************************

#define ACC_REPORT   0x01   // accel report, see 6.5.9
#define GYRO_REPORT  0x02   // gyro report, see 6.5.13
#define MAG_REPORT   0x03   // magneto report, see 6.5.16
#define LAC_REPORT   0x04   // linear accel report, see 6.5.10
#define QUAT_REPORT  0x05   // quaternion report, see 6.5.18
#define TIME_REPORT  0xFB   // time report, see 7.2.1

static const uint8_t req_acc[]  = {21, 0, 2, 0, 0xFD, ACC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_gyro[] = {21, 0, 2, 0, 0xFD, GYRO_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_mag[]  = {21, 0, 2, 0, 0xFD, MAG_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_lac[]  = {21, 0, 2, 0, 0xFD, LAC_REPORT,  0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t req_quat[] = {21, 0, 2, 0, 0xFD, QUAT_REPORT, 0, 0, 0, (REPORT_TIME>>0)&255, (REPORT_TIME>>8)&255, (REPORT_TIME>>16)&255, (REPORT_TIME>>24)&255, 0, 0, 0, 0, 0, 0, 0, 0};

static const struct TxQueue                 // list of pending tx messages
{
  const uint8_t length;
  const uint8_t *message;
} txqueue[] =
{
 #if ALL_REPORTS
  {sizeof(req_acc),  req_acc},
  {sizeof(req_gyro), req_gyro},
  {sizeof(req_mag),  req_mag},
  {sizeof(req_lac),  req_lac},
 #endif
  {sizeof(req_quat), req_quat},
};

uint8_t txqueue_msg[BNOs];                  // next message to send
uint8_t txqueue_pos[BNOs];                  // next byte to send

static uint8_t recv(uint8_t bno)            // receive one byte while simultaneously sending one byte from current tx message (or zero if no tx message)
{
  uint8_t tx = 0;
  if (txqueue_msg[bno] < numberof(txqueue)) // if more tx messages to send
  {
    if (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // if current tx message is incomplete
    {
      tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];  // byte to send
      //if (DEBUG) {Serial.print(" +");  Serial.print(tx,HEX);}
    }
  }
  uint8_t rx = SPI.transfer(tx);            // send tx, receive rx
  //if (DEBUG) {Serial.print(" ");  Serial.print(rx,HEX);}
  return rx;
}

static void flush(uint8_t bno)              // send all pending tx message bytes, then advance to next tx message
{
  if (txqueue_msg[bno] >= numberof(txqueue)) // if no more tx messages to send
    return;
  if (txqueue_pos[bno])                     // if current tx message is underway
  {
    while (txqueue_pos[bno] < txqueue[txqueue_msg[bno]].length)  // while tx message is incomplete
    {
      uint8_t tx = txqueue[txqueue_msg[bno]].message[txqueue_pos[bno]++];
      SPI.transfer(tx);                     // send tx byte, discard rx byte
      //if (DEBUG) {Serial.print(" +");  Serial.print(tx,HEX);}
    }
    txqueue_msg[bno]++;                     // prepare next tx message
    txqueue_pos[bno] = 0;
  }
}

static const decltype(pinRST) CS[]  = {pinsCS};   // array of CS pin numbers for all BNOs
static const decltype(pinRST) INT[] = {pinsINT};  // array of INT pin numbers for all BNOs

static void bno_select(uint8_t bno)         // select SPI port of desired BNO
{
  digitalWrite(CS[bno], LOW);
  pinMode(CS[bno], OUTPUT);
}

static void bno_deselect(uint8_t bno)       // deselect SPI port of desired BNO
{
  digitalWrite(CS[bno], HIGH);
  pinMode(CS[bno], OUTPUT);
}

// ******************************************
// **  Check for and parse sensor reports  **
// ******************************************

static uint16_t millis_last __attribute__((unused));  // one second timer

static void check_report(uint8_t bno)
{
  int16_t length;
  uint8_t channel __attribute__((unused));
  uint8_t seqnum[BNOs] __attribute__((unused));
  static uint8_t seqnum_next[BNOs] __attribute__((unused));
  static uint16_t seqnum_errors[BNOs] __attribute__((unused));
  static uint16_t acc_reports[BNOs] __attribute__((unused));
  static uint16_t gyro_reports[BNOs] __attribute__((unused));
  static uint16_t mag_reports[BNOs] __attribute__((unused));
  static uint16_t lac_reports[BNOs] __attribute__((unused));
  static uint16_t quat_reports[BNOs] __attribute__((unused));

 // if (DEBUG) {Serial.print("SHTP");}
  bno_select(bno);
  length = recv(bno);                       // length LSB
  length |= (uint16_t)recv(bno) << 8;       // length MSB
  length &= 0x7FFF;                         // ignore continuation flag
  channel = recv(bno);                      // channel number
  seqnum[bno] = recv(bno);                  // sequence number (ignore)
  length -= 4;                              // done reading SHTP Header

  if (length <= 0 || length > 1000)         // if null/bad/degenerate SHTP header
  {
    //if (DEBUG) {Serial.println(" What?");}
    flush(bno);
    bno_deselect(bno);
    return;
  }
  //if (DEBUG) {Serial.print(" L=");  Serial.print(length,HEX);}
  //if (DEBUG) {Serial.print(" C=");  Serial.println(channel,HEX);}

  #if STATS
    if (channel==3)
    {
      seqnum_errors[bno] += (seqnum[bno] != seqnum_next[bno]);
      seqnum_next[bno] = seqnum[bno] + 1;
    }
  #endif

  while (length)                            // while more reports in cargo
  {
    uint8_t buf[20];                        // report buffer, big enough for longest interesting report (uninteresting reports will be ignored)
    uint16_t n = 0;                         // index into report buffer

    buf[n++] = recv(bno);                   // first byte of report
    length--;

    // known reports
    if (channel==3 && buf[0]==TIME_REPORT && length >= 5-1)
    {
      for (uint8_t n=1; n<5; n++)           // read remainder of report
      {
        buf[n] = recv(bno);
        length--;
      }
     // if (DEBUG) {Serial.println(" Time");}
      continue;
    }
    #if ALL_REPORTS
      if (channel==3 && buf[0]==ACC_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        iax[bno] = *(int16_t*)&buf[4];
        iay[bno] = *(int16_t*)&buf[6];
        iaz[bno] = *(int16_t*)&buf[8];
       // if (DEBUG) {Serial.println(" Acc");}
        if (STATS) {acc_reports[bno]++;}
        continue;
      }
      if (channel==3 && buf[0]==GYRO_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        igx[bno] = *(int16_t*)&buf[4];
        igy[bno] = *(int16_t*)&buf[6];
        igz[bno] = *(int16_t*)&buf[8];
        //if (DEBUG) {Serial.println(" Gyro");}
        if (STATS) {gyro_reports[bno]++;}
        continue;
      }
      if (channel==3 && buf[0]==MAG_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        imx[bno] = *(int16_t*)&buf[4];
        imy[bno] = *(int16_t*)&buf[6];
        imz[bno] = *(int16_t*)&buf[8];
       // if (DEBUG) {Serial.println(" Mag");}
        if (STATS) {mag_reports[bno]++;}
        continue;
      }
      if (channel==3 && buf[0]==LAC_REPORT && length >= 10-1)
      {
        for (uint8_t n=1; n<10; n++)        // read remainder of report
        {
          buf[n] = recv(bno);
          length--;
        }
        ilx[bno] = *(int16_t*)&buf[4];
        ily[bno] = *(int16_t*)&buf[6];
        ilz[bno] = *(int16_t*)&buf[8];
      //  if (DEBUG) {Serial.println(" Lac");}
        if (STATS) {lac_reports[bno]++;}
        continue;
      }
    #endif
    if (channel==3 && buf[0]==QUAT_REPORT && length >= 14-1)
    {
      for (uint8_t n=1; n<14; n++)          // read remainder of report
      {
        buf[n] = recv(bno);
        length--;
      }
      iqw[bno] = *(int16_t*)&buf[10];
      iqx[bno] = *(int16_t*)&buf[4];
      iqy[bno] = *(int16_t*)&buf[6];
      iqz[bno] = *(int16_t*)&buf[8];
      //if (DEBUG) {Serial.println(" Quat");}
      if (STATS) {quat_reports[bno]++;}
      if (DATAOUT) {output_data(bno);}      // output data message
      continue;
    }

    // unwanted reports
    if (channel==0 && buf[0]==0)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
     // if (DEBUG) {Serial.println(" Advert");}
      continue;
    }
    if (channel==1 && buf[0]==1)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
      ///if (DEBUG) {Serial.println(" ExeRst");}
      continue;
    }
    if (channel==2 && buf[0]==0xF1)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
     // if (DEBUG) {Serial.println(" CmdResp");}
      continue;
    }
    if (channel==2 && buf[0]==0xFC)
    {
      while (length)                        // discard remainder of cargo
      {
        recv(bno);
        length--;
      }
      //if (DEBUG) {Serial.println(" GetFeatResp");}
      continue;
    }
    while (length)                          // discard remainder of cargo
    {
      recv(bno);
      length--;
    }
    //if (DEBUG) {Serial.println(" Unk:");  Serial.print(channel,HEX);  Serial.print(":");  Serial.println(buf[0],HEX);}
    continue;
  }
  flush(bno);
  bno_deselect(bno);

  #if STATS
    if ((uint16_t)(millis() - millis_last) > 1000)
    {
      millis_last += 1000;                  // for next second

      for (uint8_t bno=0; bno<BNOs; bno++)  // for each BNO display its various reports per second
      {
        Serial.print("Bno");  Serial.print(bno);  Serial.print("rpts: ");
        Serial.print(acc_reports[bno]);  Serial.print(",");
        Serial.print(gyro_reports[bno]); Serial.print(",");
        Serial.print(mag_reports[bno]);  Serial.print(",");
        // Serial.print(lac_reports[bno]);  Serial.print(",");
        // Serial.print(quat_reports[bno]); Serial.print(" ");
        Serial.println();
        acc_reports[bno]  = 0;
        gyro_reports[bno] = 0;
        mag_reports[bno]  = 0;
        lac_reports[bno]  = 0;
        quat_reports[bno] = 0;
      }

      Serial.print("SeqErrs: ");
      for (uint8_t bno=0; bno<BNOs; bno++)  // display sequence errors per second
      {
        Serial.print(seqnum_errors[bno]);  Serial.print(" ");
        seqnum_errors[bno] = 0;
      }
      Serial.println();
    }
  #endif

  return;
}

// **********************
// **  Setup and Loop  **
// **********************

uint8_t volatile int_count[BNOs];
uint8_t          int_counted[BNOs];

// the number of isr functions defined here must equal BNOs (I'd like to automate this with macros but don't know how)
  static void isr0(void) { int_count[0]++; }
  static void isr1(void) { int_count[1]++; }
  static void isr2(void) { int_count[2]++; }
  static void isr3(void) { int_count[3]++; }
  static void isr4(void) { int_count[4]++; }
  static void isr5(void) { int_count[5]++; }
  static void isr6(void) { int_count[6]++; }
  static void isr7(void) { int_count[7]++; }
  static void (*isr[])() = {isr0,isr1,isr2,isr3,isr4,isr5,isr6,isr7};

void setup()
{
  Serial.begin(SERIAL_BAUD);                // initialize serial
  while (!Serial);
  Serial.println("\nRunning...");

  for (uint8_t bno=0; bno<BNOs; bno++)      // disable all BNOs
    bno_deselect(bno);

  SPI.begin();                              // initialize SPI, however it sets SCK and MOSI low which is wrong for SPI_MODE3
  SPI.beginTransaction(SPI_SETTINGS);       // preset SPI signals (this is SPI.begin()'s job, not something I should have to do)

  pinMode(pinRST,OUTPUT);                   // reset all BNOs
  digitalWrite(pinRST,LOW);
  delay(2);                                 // because 1 sometimes gets truncated
  digitalWrite(pinRST,HIGH);
  delay(300);                               // below 145 fails, above 150 works

  for (uint8_t bno=0; bno<BNOs; bno++)
  {
    pinMode(INT[bno], INPUT);
    attachInterrupt(digitalPinToInterrupt(INT[bno]), isr[bno], FALLING);
  }

  millis_last = millis();
}

void loop()
{
  Serial.print("TEST STRING \n");
  for (uint8_t bno=0; bno<BNOs; bno++)      // check for reports from all BNOs
  {
    if (int_count[bno] != int_counted[bno]) // if transfer request
    {
      int_counted[bno] = int_count[bno];    // acknowledge all interrupt(s)
      //if (DEBUG) {Serial.print(" BNO=");  Serial.println(bno);}
      check_report(bno);
    }
  }
}