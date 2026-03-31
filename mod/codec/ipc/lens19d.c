#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "fw/comm/inc/serial.h"
#include "cfg.h"
#include "lens.h"
#include "mpp.h"
#include "fw/libaf/inc/af_ptz.h"

#define DEBUG 0
#define SUM6(buf) do{buf[6] = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5])&0xFF;}while(0)
extern int dzoom_plus;

static int _sensor_flag = 0;
static int _flash_emmc  = 0;

enum {
  LENS_UART_LDM = 0,
  LENS_UART_ZYS = 1
}; static int _uart_type = LENS_UART_LDM;//LENS_UART_ZYS;//LENS_UART_LDM;

enum {
  LENS_TYPE_UART,  // 0: fixed-lens & uart-zoom
  LENS_TYPE_SONY,  // 1: sony-zoom
  LENS_TYPE_GPIO,  // 2: computar
  LENS_TYPE_HIVIEW,// 3: hiview-zoom
}; static int _lens_type = LENS_TYPE_UART;

enum {
  IRCUT_TYPE_MANUAL, //0: manual
  IRCUT_TYPE_AUTO,   //1: auto
  IRCUT_TYPE_CDS,    //2: cds
}; static int _ircut_type = IRCUT_TYPE_MANUAL;

enum {
  IRCUT_CTL_EDGE,  // 0: [0_1-0_0 -> 1_0-0_0]
  IRCUT_CTL_LEVEL, // 1: [0_0 -> 1_1]
}; static int _ircut_ctl  = IRCUT_CTL_EDGE;

enum {
  LAMP_IR = 0,
  LAMP_WHITE = 1,
}; static int _lamp_type = LAMP_IR;//LAMP_IR;


enum {
  IRCUT_REV_0,
  IRCUT_REV_1,
}; static int _ircut_rev = IRCUT_REV_0;


static gsf_lens_ini_t _ini;
static pthread_t serial_tid;
static int serial_fd[2] = {-1, -1}, last_cmd[2] = {GSF_LENS_STOP, GSF_LENS_STOP};
static int _zoomValue = 0, _cdsValue = 0, _dayNight = 0; // 0: day,  1: night;
static void* serial_task(void *parm);
static int pelco_d_write(char *cmd, int size);
static int ptz_led_set(int stat);


#if defined(GSF_CPU_3519d)

#warning "3519d lens is implemented"

#define IRCUT_CTL_TYPE(sns) (strstr(sns, "imx585") || strstr(sns, "imx482") || strstr(sns, "imx678") || strstr(sns, "imx664"))?IRCUT_CTL_LEVEL:IRCUT_CTL_EDGE

//#5-1, 2-0; IRCUT1
#define IRCUT0_INIT() do {\
    system("bspmm 0x01026006C 0;bspmm 0x010260000 0;echo 41 > /sys/class/gpio/export;echo 16 > /sys/class/gpio/export;");\
    system("echo low > /sys/class/gpio/gpio41/direction;echo low > /sys/class/gpio/gpio16/direction");\
  }while(0)

#define __IRCUT0_DAY(ctl) do {\
    if(ctl == IRCUT_CTL_LEVEL)\
      system("echo 0 > /sys/class/gpio/gpio41/value;echo 0 > /sys/class/gpio/gpio16/value;");\
    else \
      system("echo 0 > /sys/class/gpio/gpio41/value;echo 1 > /sys/class/gpio/gpio16/value;sleep 0.1;echo 0 > /sys/class/gpio/gpio16/value");\
  }while(0)

#define __IRCUT0_NIGHT(ctl) do {\
    if(ctl == IRCUT_CTL_LEVEL)\
      system("echo 1 > /sys/class/gpio/gpio41/value;echo 1 > /sys/class/gpio/gpio16/value;");\
    else \
      system("echo 1 > /sys/class/gpio/gpio41/value;echo 0 > /sys/class/gpio/gpio16/value;sleep 0.1;echo 0 > /sys/class/gpio/gpio41/value");\
  }while(0)

//#10-2 4-4; IRCUT2
#define IRCUT1_INIT() do {\
    system("bspmm 0x0179F000C 1;bspmm 0x010260058 1;echo 82 > /sys/class/gpio/export;echo 36 > /sys/class/gpio/export;");\
    system("echo low > /sys/class/gpio/gpio82/direction;echo low > /sys/class/gpio/gpio36/direction");\
  }while(0)

#define __IRCUT1_DAY(ctl) do {\
    if(ctl == IRCUT_CTL_LEVEL)\
      system("echo 0 > /sys/class/gpio/gpio82/value;echo 0 > /sys/class/gpio/gpio36/value;");\
    else \
      system("echo 0 > /sys/class/gpio/gpio82/value;echo 1 > /sys/class/gpio/gpio36/value;sleep 0.1;echo 0 > /sys/class/gpio/gpio36/value");\
  }while(0)

#define __IRCUT1_NIGHT(ctl) do {\
    if(ctl == IRCUT_CTL_LEVEL)\
      system("echo 1 > /sys/class/gpio/gpio82/value;echo 1 > /sys/class/gpio/gpio36/value;");\
    else \
      system("echo 1 > /sys/class/gpio/gpio82/value;echo 0 > /sys/class/gpio/gpio36/value;sleep 0.1;echo 0 > /sys/class/gpio/gpio82/value");\
  }while(0)

//ircut reversed
#define IRCUT0_DAY(ctl) do{ \
    if(_ircut_rev) __IRCUT0_NIGHT(ctl); else __IRCUT0_DAY(ctl);\
  }while(0)

#define IRCUT0_NIGHT(ctl) do{ \
    if(_ircut_rev) __IRCUT0_DAY(ctl); else __IRCUT0_NIGHT(ctl);\
  }while(0)

#define IRCUT1_DAY(ctl) do{ \
    if(_ircut_rev) __IRCUT1_NIGHT(ctl); else __IRCUT1_DAY(ctl);\
  }while(0)

#define IRCUT1_NIGHT(ctl) do{ \
    if(_ircut_rev) __IRCUT1_DAY(ctl); else __IRCUT1_NIGHT(ctl);\
  }while(0)

//#1-4  Lamp0
#define LAMP0_INIT() do {\
    system("bspmm 0x00EFF004C 0;echo 12 > /sys/class/gpio/export");\
    system("echo high > /sys/class/gpio/gpio12/direction");\
  }while(0)

#define LAMP0_DAY() do {\
      system("echo 1 > /sys/class/gpio/gpio12/value");\
  }while(0)

#define LAMP0_NIGHT() do {\
      system("echo 0 > /sys/class/gpio/gpio12/value");\
  }while(0)



static int serial_set_timeout(int fd, int to_sec)
{
  if(fd < 0)
    return -1;
    
  struct termios options;
  tcgetattr(fd, &options);
  options.c_cc[VTIME] = to_sec*10; // timeout
  options.c_cc[VMIN] = 0;     // none
  tcsetattr(fd, TCSANOW, &options);
  return 0;
}


static int flash_is_emmc()
{
  int ret = 0;
  char str[256];
  sprintf(str, "%s", "cat /proc/cmdline  | grep mmcblk");
  FILE* fd = popen(str, "r");
  if (fd && fgets(str, sizeof(str), fd))
  {
    ret = strstr(str, "mmcblk")?1:0;
    pclose(fd);
  }
  return ret;
}

#include <arpa/inet.h>
#include <sys/socket.h>
static int af_ctl(char *buf, int size)
{
  int ret = 0;
  struct sockaddr_in to_addr;
  memset(&to_addr, 0, sizeof(struct sockaddr_in));
  to_addr.sin_family = AF_INET;
  to_addr.sin_port = htons(3000);
  to_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  
  static int sock = -1;

  if(sock < 0)
  {
    sock = socket(PF_INET, SOCK_DGRAM, 0);
    
    struct timeval tv;
  	tv.tv_sec  = 3;
  	tv.tv_usec = 0;
  	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }
  
  ret = sendto(sock, buf, size, 0, (struct sockaddr*)&to_addr, sizeof(to_addr));
  printf("sendto ret:%d\n", ret);
  return ret;
}


int lens19d_lens_init(gsf_lens_ini_t *ini)
{
  _ini = *ini;
  _sensor_flag = (strstr(_ini.sns, "imx") ||strstr(_ini.sns, "os"))?1:0;
  _flash_emmc  = flash_is_emmc();
  _lens_type   = codec_ipc.lenscfg.lens;
  _ircut_type  = codec_ipc.lenscfg.ircut;
  _ircut_ctl   = IRCUT_CTL_TYPE(_ini.sns);
  _ircut_rev   = codec_ipc.lenscfg.ircut_rev;
  
  printf("sensor:%s, flash_emmc:%d, lens_type:%d, "
         "ircut_type:%d, ircut_ctl:%d, ircut_rev:%d\n"
        , _ini.sns, _flash_emmc, _lens_type
        , _ircut_type, _ircut_ctl, _ircut_rev);
    
  if(!_sensor_flag)
  {
    if(strstr(_ini.sns, "yuv422cam"))
    {
      //MIPI-POWER
      system("bspmm 0x011120214 0x1201;"); //gpio9_4
      system("echo 76 > /sys/class/gpio/export;echo high > /sys/class/gpio/gpio76/direction;");

      //LVDS-POWER
      system("bspmm 0x011120218 0x1201;"); //gpio9_5
      system("echo 77 > /sys/class/gpio/export;echo high > /sys/class/gpio/gpio77/direction;");
    }
    return 0;
  }
   
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    //th1280-POWER
    
    printf("LENS_TYPE_HIVIEW Don't init IRCUT\n");
    return 0;
  } 
    
  IRCUT0_INIT();
  IRCUT0_DAY(_ircut_ctl);

  IRCUT1_INIT();
  IRCUT1_DAY(_ircut_ctl);

  LAMP0_INIT();

  return 0;
}

static int ir_cb(int ViPipe, int dayNight, void* uargs)
{
  _dayNight = dayNight;
  
  if(_lens_type == LENS_TYPE_HIVIEW)
  { 
    printf("LENS_TYPE_HIVIEW set IRCUT\n");
    return 0;
  }
  else
  {
    if(dayNight)
    {   
      if(_lamp_type == LAMP_IR) IRCUT0_NIGHT(_ircut_ctl);
      LAMP0_NIGHT();
    }
    else
    {  
      if(_lamp_type == LAMP_IR) IRCUT0_DAY(_ircut_ctl);
      LAMP0_DAY();
    } 
  }
  
  printf("ViPipe:%d, IR night:%d\n", ViPipe, dayNight);
  return 0;
}

//return  0: day, 1: night;
static int cds_cb(int ViPipe, void* uargs)
{
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    //printf("LENS_TYPE_HIVIEW get CDS\n");
    return _cdsValue; 
  } 
   
  int value = 0;
 
  FILE* fp = fopen("/sys/class/gpio/gpio13/value", "rb+");
  if(fp)
  {
    unsigned char buf[10] = {0};
    fread(buf, sizeof(char), sizeof(buf) - 1, fp);
    value = (buf[0] == '0')?1:0;
    fclose(fp);
  }
  else 
  {
    printf("err: open fp:%p\n", fp);
  }

  printf("ViPipe:%d, CDS night:%d\n", ViPipe, value);
  return value;
}

int lens19d_lens_ircut(int ch, int dayNight)
{
  if(!_sensor_flag)
  {  
    return -1;
  }
  
  ir_cb(ch, dayNight, NULL); 
  if(_lamp_type == LAMP_IR)
  {
    gsf_mpp_isp_ctl(0, GSF_MPP_ISP_CTL_IR, (void*)(uintptr_t)dayNight);
  }
  return 0;
}

int lens19d_uart_write(int ch, unsigned char *buf, int size)
{
  int ret = 0;

  if(serial_fd[ch] > 0)
  {
    struct timespec ts1;  
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    
    ret = write(serial_fd[ch], buf, size);
    #if DEBUG
    printf("ch:%d, ret:%d, ms:%u, buf[%02X %02X %02X %02X %02X %02X %02X %02X]\n"
        , ch, ret, (ts1.tv_sec*1000 + ts1.tv_nsec/1000000)
        , buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    #endif
	}
	return ret;
}

static int af_cb(HI_U32 Fv1, HI_U32 Fv2, HI_U32 Gain, void* uargs)
{
  unsigned char buf[8];

  if(_uart_type == LENS_UART_LDM)
  {
    HI_U32 Fv = Fv1 + Fv2;   
    //FV: A5 + 4字节AF数据高位在前 + 2字节增益(模拟增益)+ 1字节颜色(彩色/黑白)
    buf[0] = 0xa5;
    buf[1] = (Fv >> 24) & 0xFF;	
    buf[2] = (Fv >> 16) & 0xFF;
    buf[3] = (Fv >> 8) & 0xFF;
    buf[4] = Fv & 0xFF;

    buf[5] = (Gain >> 8) & 0xFF;
    buf[6] = Gain & 0xFF;

    buf[7] = _dayNight; //彩色是0 黑白是1

    int ret = gsf_uart_write(0, buf, 8);
  }
  else 
  {
    //{0xFF,0xAF,AFD1高8位,AFD2低8位, 0灭/1红外/2白光, 0-100灯亮度,sum}
    //AFD = (FV >> xxx), 确保AFD数据不溢出(16bit最高位为0)
    HI_U32 Fv = Fv2;
    buf[0] = 0xFF;
    buf[1] = 0xAF;
    buf[2] = (Fv >> 8) & 0xFF; //打印测试是否溢出
    buf[3] = (Fv >> 0) & 0xFF;  //打印测试是否溢出
    buf[4] = 0x00;
    buf[5] = 0x00;
    SUM6(buf);
    int ret = gsf_uart_write(0, buf, 7);
    //printf("Fv:%08d (Fv1:%08d, Fv2:%08d)\n", (buf[2] << 8) | (buf[3]), Fv1, Fv2);
  }
  return 0;
}

int lens19d_lens_start(int ch, char *ttyAMA)
{
  int ret = 0;
  
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    //dev/ttyAMA1 for ptz;
    gsf_uart_open(0, ttyAMA, 9600);
    
    if(strstr(_ini.sns, "imx678"))
    {
      //dev/ttyAMA4 for th1280;
      gsf_uart_open(1, "/dev/ttyAMA4", 115200);
    }
  }  
  else if(_lens_type == LENS_TYPE_GPIO)
  {
    ;
    return -1;
  }  
  else if(_lens_type == LENS_TYPE_SONY)
  {
    //dev/ttyAMA3 for yuv422cam;
    gsf_uart_open(0, ttyAMA, 9600); 
    
    if(strstr(_ini.sns, "yuv422cam"))
    {
      //dev/ttyAMA1 for sonycam;
      gsf_uart_open(1, "/dev/ttyAMA1", 9600);
    }
    return -1;
  }
  else
  {
    int baudrate = (_uart_type == LENS_UART_LDM)?115200:9600;
      
    if(gsf_uart_open(0, ttyAMA, baudrate) < 0)
    {  
      printf("open error ttyAMA:[%s]\n", ttyAMA);
    }
  }
  
  printf("%s => _sensor_flag:%d\n", __func__, _sensor_flag);
  if(!_sensor_flag)
  {  
    return -1;
  }
  
  if(_ircut_type)
  {
    gsf_mpp_ir_t ir = {.cds = (_ircut_type == IRCUT_TYPE_CDS)?cds_cb:NULL, .cb = ir_cb};
    ret = gsf_mpp_isp_ctl(0, GSF_MPP_ISP_CTL_IR, &ir);
  }
  else if (_lens_type == LENS_TYPE_HIVIEW)
  {
    gsf_mpp_ir_t ir = {.cds = cds_cb, .cb = ir_cb};
    ret = gsf_mpp_isp_ctl(0, GSF_MPP_ISP_CTL_IR, &ir);
  }

  if(ch < 0 || _lens_type == LENS_TYPE_HIVIEW)
  {
    printf("LENS_TYPE_HIVIEW DISABLE gsf_mpp_af_start()\n");
  	return 0;
  }
  
  gsf_mpp_af_t af = {
      .ViPipe = ch,
      .uargs = (void*)(uintptr_t)_uart_type, //af_alg;
      .cb = af_cb,
  };
  
  printf("%s => gsf_mpp_af_start(ch:%d, af_alg:%d)\n", __func__, ch, _uart_type);
  return gsf_mpp_af_start(&af);
}


int lens19d_lens_stop(int ch)
{
  int ret = 0;
  dzoom_plus = 0;
  
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    if(ch == 0)
    {  
      unsigned char zm_stop[]  = {0xAF, 0x01, 0x04, 0x00};
      unsigned char foc_stop[] = {0xAF, 0x10, 0x05, 0x00, 0x00};
      unsigned char *buf = (last_cmd[ch] == GSF_LENS_FOCUS)?foc_stop:zm_stop;
      af_ctl(buf, (last_cmd[ch] == GSF_LENS_FOCUS)?sizeof(foc_stop):sizeof(zm_stop));
    }
    else 
    {
      //th1280
      //变倍停   55 AA 07 03 00 07 00 00 00 00 03 F0
      //调焦停   55 AA 07 03 00 06 00 00 00 00 02 F0
      
      unsigned char zm_stop[]  = {0x55,0xAA,0x07,0x03,0x00,0x07,0x00,0x00,0x00,0x00,0x03,0xF0};
      unsigned char foc_stop[] = {0x55,0xAA,0x07,0x03,0x00,0x06,0x00,0x00,0x00,0x00,0x02,0xF0};
      unsigned char *buf = (last_cmd[ch] == GSF_LENS_FOCUS)?foc_stop:zm_stop;
      if(ch > 0 && serial_fd[1] > 0)
      {  
        ret = gsf_uart_write(1, buf, sizeof(zm_stop));
        
        if(0)//if(last_cmd[ch] == GSF_LENS_ZOOM)
        {
          usleep(100*1000);
          //自动对焦 55 AA 07 03 00 06 00 00 00 03 01 F0
          unsigned char auto_foc[] = {0x55,0xAA,0x07,0x03,0x00,0x06,0x00,0x00,0x00,0x03,0x01,0xF0};
          ret = gsf_uart_write(1, auto_foc, sizeof(auto_foc));
        }
      }
    }
    
    printf("LENS_TYPE_HIVIEW  ch:%d, stop %s\n", ch, (last_cmd[ch] == GSF_LENS_FOCUS)?"FOCUS":"ZOOM");
    last_cmd[ch] = GSF_LENS_STOP;
  }
  else if(_lens_type == LENS_TYPE_GPIO)
  {
    ;
    return 0;
  }
  else if(_lens_type == LENS_TYPE_SONY)
  {
    unsigned char buf[6] = {0x81, 0x01, 0x04, 0x07, 0x00, 0xFF};

    if(ch > 0 && serial_fd[1] > 0)
      ret = gsf_uart_write(1, buf, 6);
    else 
      ret = gsf_uart_write(0, buf, 6);
    return 0;
  }
  else 
  {
    if(_uart_type == LENS_UART_LDM)
    {  
      unsigned char buf[8] = {0xc5,0x00,0x00,0x00,0x00,0x00,0x00,0x5c}; SUM6(buf);
      ret = gsf_uart_write(0, buf, 8);
    }
    else 
    {
      unsigned char buf[7] = {0xff,0x01,0x00,0x00,0x00,0x00,0x01}; SUM6(buf);
      ret = gsf_uart_write(0, buf, 7);
    }
  }
  return 0;
}

int lens19d_lens_zoom(int ch,  int dir, int speed)
{
  int ret = 0;
  
  if(strstr(_ini.sns, "imx585") || strstr(_ini.sns, "imx482"))
  {
    dzoom_plus = (dir)?1:-1;
    return 0;
  }
  
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    if(ch == 0)
    {  
      unsigned char add[] = {0xAF, 0x01, 0x04, 0x01};
      unsigned char sub[] = {0xAF, 0x01, 0x04, 0x02};
      unsigned char *buf = (dir)?add:sub;
      af_ctl(buf, sizeof(add));
    }
    else 
    {
      //th1280;
      
      //变倍+ 55 AA 07 03 00 07 00 00 00 01 02 F0
      //变倍- 55 AA 07 03 00 07 00 00 00 02 01 F0
      unsigned char add[] = {0x55,0xAA,0x07,0x03,0x00,0x07,0x00,0x00,0x00,0x01,0x02,0xF0};
      unsigned char sub[] = {0x55,0xAA,0x07,0x03,0x00,0x07,0x00,0x00,0x00,0x02,0x01,0xF0};
      unsigned char *buf = (dir)?add:sub;
      if(ch > 0 && serial_fd[1] > 0)
        ret = gsf_uart_write(1, buf, sizeof(add));
      
    }
    last_cmd[ch] = GSF_LENS_ZOOM;
    printf("LENS_TYPE_HIVIEW ch:%d start %s\n", ch, "ZOOM");
  }  
  else if(_lens_type == LENS_TYPE_GPIO)
  {
    ;
    return 0;
  }
  else if(_lens_type == LENS_TYPE_SONY)
  {
    unsigned char add[6] = {0x81, 0x01, 0x04, 0x07, 0x25, 0xFF}; //buf[4] = 0x20 | (speed&0x0F);
    unsigned char sub[6] = {0x81, 0x01, 0x04, 0x07, 0x35, 0xFF}; //buf[4] = 0x30 | (speed&0x0F);
    unsigned char *buf = (dir)?add:sub;
    if(ch > 0 && serial_fd[1] > 0)
      ret = gsf_uart_write(1, buf, 6);
    else 
      ret = gsf_uart_write(0, buf, 6);
    return 0;
  }
  else 
  {
    if(_uart_type == LENS_UART_LDM)
    {  
    	// 派尔高D协议开始字节FF换成C5,最后补充一个字节5C组成8个字节
      unsigned char add[8] = {0xc5,0x00,0x00,0x20,0x00,0x00,0x00,0x5c}; SUM6(add);
      unsigned char sub[8] = {0xc5,0x00,0x00,0x40,0x00,0x00,0x00,0x5c}; SUM6(sub);
      unsigned char *buf = (dir)?add:sub;
      ret = gsf_uart_write(0, buf, 8);
    }
    else 
    {
      //{0xff,0x01,0x02,0x00,0x00,0x00,0x03}//光圈小
      //{0xff,0x01,0x04,0x00,0x00,0x00,0x05}//光圈大
      //{0xff,0x01,0x00,0x25,0x00,speed,sum}//变倍速度 4x speed 0-4 慢-快
      //{0xff,0x01,0x00,0x20,0x00,0x00,0x21}//变倍短
      //{0xff,0x01,0x00,0x40,0x00,0x00,0x41}//变倍长
      unsigned char add[7] = {0xff,0x01,0x00,0x20,0x00,0x00,0x21}; SUM6(add);
      unsigned char sub[7] = {0xff,0x01,0x00,0x40,0x00,0x00,0x41}; SUM6(sub);
      unsigned char *buf = (dir)?add:sub;
      ret = gsf_uart_write(0, buf, 7);
    }
  }
  return 0;
}

int lens19d_lens_focus(int ch, int dir, int speed)
{
  int ret = 0;
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    if(ch == 0)
    {  
      unsigned char add[] = {0xAF, 0x10, 0x05, 0x00, 0x04};
      unsigned char sub[] = {0xAF, 0x10, 0x05, 0x01, 0x04};
      unsigned char *buf = (dir)?add:sub;
      af_ctl(buf, sizeof(add));
    }
    else 
    {
      //th1280
      //远焦+ 55 AA 07 03 00 06 00 00 00 01 03 F0
      //远焦- 55 AA 07 03 00 06 00 00 00 02 00 F0
      //手动调焦速度(1~10) 55 AA [07 03 00 02 00 00 00 XX] XOR F0
      unsigned char add[] = {0x55,0xAA,0x07,0x03,0x00,0x06,0x00,0x00,0x00,0x01,0x03,0xF0};
      unsigned char sub[] = {0x55,0xAA,0x07,0x03,0x00,0x06,0x00,0x00,0x00,0x02,0x00,0xF0};
      unsigned char *buf = (dir)?add:sub;
      if(ch > 0 && serial_fd[1] > 0)
        ret = gsf_uart_write(1, buf, sizeof(add));
    }
    last_cmd[ch] = GSF_LENS_FOCUS;
    printf("LENS_TYPE_HIVIEW  ch:%d, start %s\n", ch, "FOCUS");
  }  
  else if(_lens_type == LENS_TYPE_GPIO)
  {
    ;
    return 0;
  }
  else if(_lens_type == LENS_TYPE_SONY)
  {
    unsigned char add[6] = {0x81, 0x01, 0x04, 0x08, 0x25, 0xFF};
    unsigned char sub[6] = {0x81, 0x01, 0x04, 0x08, 0x35, 0xFF};
    unsigned char *buf = (dir)?add:sub;    
    if(ch > 0 && serial_fd[1] > 0)
      ret = gsf_uart_write(1, buf, 6);
    else 
      ret = gsf_uart_write(0, buf, 6);
    
    return 0;
  }
  else 
  {
    if(_uart_type == LENS_UART_LDM)
    {  
    	// 派尔高D协议开始字节FF换成C5,最后补充一个字节5C组成8个字节
      unsigned char add[8] = {0xc5,0x00,0x01,0x00,0x00,0x00,0x00,0x5c}; SUM6(add);
      unsigned char sub[8] = {0xc5,0x00,0x00,0x80,0x00,0x00,0x00,0x5c}; SUM6(sub);
      unsigned char *buf = (dir)?add:sub;
      ret = gsf_uart_write(0, buf, 8);
    }
    else 
    {
      unsigned char add[7] = {0xff,0x01,0x01,0x00,0x00,0x00,0x02}; SUM6(add);
      unsigned char sub[7] = {0xff,0x01,0x00,0x80,0x00,0x00,0x81}; SUM6(sub);
      unsigned char *buf = (dir)?add:sub;
      ret = gsf_uart_write(0, buf, 7);
    }

  }
  return 0;
}

int lens19d_lens_cal(int ch)
{
	// lens calibration
  if(_lens_type == LENS_TYPE_HIVIEW)
  {
    unsigned char buf[] = {0xAF, 0x10, 0x04};
    af_ctl(buf, sizeof(buf));
  }  
  else
  {
    if(_uart_type == LENS_UART_LDM)
    {  
      unsigned char buf[8] = {0xc5,0x00,0x00,0x07,0x00,250,0x00,0x5c}; SUM6(buf);
      int ret = gsf_uart_write(0, buf, 8);
      usleep(100*1000);
      ret |= gsf_uart_write(0, buf, 8);
    }
  }
  return 0;
}

int lens19d_uart_open(int ch, char *ttyAMA, int baudrate)
{
  if(strstr(ttyAMA, "ttyAMA3"))
    system("bspmm 0x0102600E0 2; bspmm 0x0102600E4 2;"); //UART3 MUX
  else if(strstr(ttyAMA, "ttyAMA1"))
    system("bspmm 0x0102600D8 1; bspmm 0x0102600DC 1;"); //UART1 MUX 
  else if(strstr(ttyAMA, "ttyAMA4"))
    system("bspmm 0x010260008 0x02; bspmm 0x01026000C 0x02;"); //UART4 MUX 
  else  
    return -1;    
  
  if(!ttyAMA || strlen(ttyAMA) < 1)
    return -1;
  //blocking read;
  serial_fd[ch] = open(ttyAMA, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
  if (serial_fd[ch] < 0)
  {
      return -2;
  }
	// set serial param
  if(serial_set_param(serial_fd[ch], baudrate, 0, 1, 8) < 0)
  {
    return -3;
  }

  //serial_set_timeout(serial_fd[ch], 5);
  printf("baudrate:%d\n", baudrate);
  return pthread_create(&serial_tid, NULL, serial_task, (void*)(uintptr_t)ch);
}

static void* serial_task_zys(void *parm)
{
  //倍数返回信息方式1
  //{0xff,0x0d,0x00,0x07,0x00,0x01,0x15,}//倍数1 校验和不含数据头0xff
  //{0xff,0x0d,0x00,0x07,0x00,0x02,0x16,}//倍数2 校验和不含数据头0xff

  return NULL;
}

static void* serial_task_ldm(void *parm)
{
  int ret = 0;
  unsigned short cmd = 0;
  unsigned char buf[4+256] = {0};
  int ch = (int)(uintptr_t)parm;
  
  while(serial_fd[ch] > 0)
  {
    //OSD: 0xEF 0x01 0x00 len 0x00(foreground) 0x02(index) row col char0~charN
    buf[0] = buf[1] = buf[2] = buf[3] = 0;
    ret = read(serial_fd[ch], buf, 4);
    if(buf[0] != 0xef || buf[1] != 0x01 || buf[2] != 0x00)
    {
      #if DEBUG
      printf("err hdr ch:%d, ret:%d, buf[%02X %02X %02X %02X]\n"
          , ch, ret, buf[0], buf[1], buf[2], buf[3]);
      #endif
      usleep(10);
      continue;
    }
    
    ret += read(serial_fd[ch], &buf[4], buf[3]);
    #if DEBUG
    int i = 0;
    unsigned char bufstr[sizeof(buf)*3] = {0};
    for(i = 0; i < ret && i < sizeof(buf); i++)
    {
      char token[4];
      sprintf(token, "%02X ", buf[i]);
      strcat(bufstr, token);
    }
    printf("ok read ch:%d, ret:%d, buf[%s]\n", ch, ret, bufstr);
    #endif
  }
  return NULL;
}


static void* serial_task_ptz(void *parm)
{
  int ret = 0;
  unsigned short cmd = 0;
  unsigned char buf[256] = {0};
  int ch = (int)(uintptr_t)parm;
  while(serial_fd[ch] > 0)
  {
    usleep(10*1000);
  }
  return NULL;
}

static void* serial_task_th1280(void *parm)
{
  int ret = 0, len = 0;
  unsigned char buf[64] = {0};
  int ch = (int)(uintptr_t)parm;
  
  //55 AA [07 (02 01 08 00 00 00 01)] XOR F0
  
  while(serial_fd[ch] > 0)
  {
HEAD:    
    ret = buf[0] = buf[1] = buf[2] = 0;
    while(ret < 3)
    {
      int r = read(serial_fd[ch], buf, 3-ret);
      if(r < 0)
        goto HEAD;
      else if(r == 0)
        usleep(10);
      ret += r;    
    }
    if(ret < 0 || buf[0] != 0x55 || buf[1] != 0xAA || buf[2] == 0)
    {
      #if DEBUG
      printf("err hdr ch:%d, ret:%d, buf[%02X %02X %02X]\n", ch, ret, buf[0], buf[1], buf[2]);
      #endif
      usleep(10);
      continue;
    }
BODY:
    len = 2 + 1 + buf[2] + 2;    
    while(ret < len)
    {
      int r = read(serial_fd[ch], &buf[ret], len-ret);
      if(r < 0)
        goto HEAD;
       else if (r == 0)
        usleep(10);
        
       ret += r;
    }

    #if DEBUG
    int i = 0;
    unsigned char bufstr[sizeof(buf)*3] = {0};
    for(i = 0; i < ret && i < sizeof(buf); i++)
    {
      char token[4];
      sprintf(token, "%02X ", buf[i]);
      strcat(bufstr, token);
    }
    printf("ok read ch:%d, ret:%d, buf[%s]\n", ch, ret, bufstr);
    #endif
    
    if(buf[3] == 0xA0 && buf[4] == 0x09)
    {
      /*--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
      |0  ......    4| 5 startup  | 6 zoom   | 7 zoom-direct     | 8 af-status      |9,10 zoom-val  |11 hfov.int| 12 hfov.point| 13 vfov.int| 14 vfov.point| ..... 57  58  |
      |---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
      |55 AA 36 A0 09| 0~2:自检中 | 0：变倍停| 0x00：切换中      | 0x00：聚焦结束   |焦距值(x10)     | HFOV_int  | HFOV_point   | VFOV_int   | VFOV_point   |       XOR 0xF0|  
      |              | 3:自检完成 | 1：变倍+ | 0x0A：达到指定焦距| 0x03：自动聚焦中 |如50mm即[01,f4] | HFOV整数  | HFOV小数     | VFOV整数   | VFOV小数     |               |
      |              |            |  2：变倍-|                   |                  |高位在前低位在后|           |              |            |              |               |
      |---------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
      printf("ch:%d, startup:%d, zoom:%d, zoom-direct:%d, af-status:%d, zoom-val:%d fov:[%02d.%02d, %02d.%02d]\n"
              , ch, buf[5], buf[6], buf[7], buf[8], (((buf[9]&0xff)<<8)|(buf[10]&0xff)), buf[11], buf[12], buf[13], buf[14]);
    }
  }
  return NULL;
}

static void* serial_task(void *parm)
{
  int ch = (int)(uintptr_t)parm;
  
  if(ch == 0)
  {
    if(_lens_type == LENS_TYPE_HIVIEW)
      return serial_task_ptz(parm);
    else if(_uart_type == LENS_UART_LDM)
      return serial_task_ldm(parm);
    else 
      return serial_task_zys(parm);
  }
  else 
  {
    return serial_task_th1280(parm);
  }

}


int lens19d_lens_ptz(int ch, gsf_lens_t *lens)
{
  int ret = 0;
  
  if(_lens_type != LENS_TYPE_HIVIEW)  
    return 0;
  
  unsigned char buf[4];
  buf[0] = lens->cmd;
  buf[1] = lens->arg1;
  buf[2] = lens->arg2;
  buf[3] = 0x00;
  ret = pelco_d_write(buf, sizeof(buf));
  printf("pelco_d_write ret:%d, cmd:%d[6:stop,7:up,8:down,9:left,10:right]\n", ret, lens->cmd);
  
  switch(lens->cmd)
  { 
    case GSF_PTZ_STOP: 
    {  
      //th1280
      //auto_foc
    }
    break;
    case GSF_PTZ_UP:
    case GSF_PTZ_DOWN:
    case GSF_PTZ_LEFT:
    case GSF_PTZ_RIGHT:
    break;
  }
  return 0;
}

static int pelco_d_write(char *cmd, int size)
{
  int ret = -1;
  
  /*
  地址码: 0x01, 速度:0x20[0x00-0x3F]
  校验码: MOD[(字节2 + 字节3 + 字节4 + 字节5 + 字节6)/100H]
  FF 01 00 00 00 00 01//停
  FF 01 00 08 00 20 29//上
  FF 01 00 10 00 20 31//下
  FF 01 00 04 20 00 25//左
  FF 01 00 02 20 00 23//右
  FF 01 00 0C 20 20 4D//左上
  FF 01 00 0A 20 20 4B//右上
  FF 01 00 14 20 20 55//左下
  FF 01 00 12 20 20 53//右下
  FF 01 00 03 00 01 05 //设置1号预置位
  FF 01 00 07 00 01 09 //调用1号预置位
  FF 01 00 05 00 01 07 //删除1号预置位
  */
  
  switch(cmd[0])
  {
    case GSF_PTZ_STOP:
    {
      unsigned char buf[7] = {0xff,0x01,0x00,0x00,0x00,0x00,0x00};
      buf[6] = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5])&0xFF;
      ret = gsf_uart_write(0, buf, sizeof(buf));
    }  
    break;
    case GSF_PTZ_UP:
    {
      unsigned char buf[7] = {0xff,0x01,0x00,0x08,0x00,0x20,0x00};
      buf[6] = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5])&0xFF;
      ret = gsf_uart_write(0, buf, sizeof(buf));
    }
    break;    
    case GSF_PTZ_DOWN:
    {
      unsigned char buf[7] = {0xff,0x01,0x00,0x10,0x00,0x20,0x00};
      buf[6] = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5])&0xFF;
      ret = gsf_uart_write(0, buf, sizeof(buf));
    }
    break;  
    case GSF_PTZ_LEFT:
    {
      unsigned char buf[7] = {0xff,0x01,0x00,0x04,0x20,0x00,0x00};
      buf[6] = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5])&0xFF;
      ret = gsf_uart_write(0, buf, sizeof(buf));
    }
    break;    
    case GSF_PTZ_RIGHT:
    {
      unsigned char buf[7] = {0xff,0x01,0x00,0x02,0x20,0x00,0x00};
      buf[6] = (buf[1]+buf[2]+buf[3]+buf[4]+buf[5])&0xFF;
      ret = gsf_uart_write(0, buf, sizeof(buf));
    }
    break;
  }
  return ret;
}

int (*gsf_lens_start)(int ch, char *ttyAMA) = lens19d_lens_start;
int (*gsf_lens_ircut)(int ch, int dayNight) = lens19d_lens_ircut;
int (*gsf_lens_zoom)(int ch,  int dir, int speed) = lens19d_lens_zoom;
int (*gsf_lens_focus)(int ch, int dir, int speed) = lens19d_lens_focus;
int (*gsf_lens_stop)(int ch) = lens19d_lens_stop;
int (*gsf_lens_cal)(int ch) = lens19d_lens_cal;
int (*gsf_uart_open)(int ch, char *ttyAMA, int baudrate) = lens19d_uart_open;
int (*gsf_uart_write)(int ch, unsigned char *buf, int size) = lens19d_uart_write;
int (*gsf_lens_init)(gsf_lens_ini_t *ini) = lens19d_lens_init;
int (*gsf_lens_ptz)(int ch, gsf_lens_t *lens) = lens19d_lens_ptz;

#endif