#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <math.h>

#include <atom.h>
#include <atomsem.h>
#include <atomqueue.h>
#include <atomtimer.h>

#include "hw.h"


#define SDELAY 512 
volatile int64_t gpos=0;
int16_t offset=0;
int8_t step=0;
uint16_t position=0;
int16_t ppoll=0;
volatile int16_t diff=0;
void run_motor(void);
void setstep(uint8_t s);
int spi_poll2(void);
void spi_poll1(void);

int spi_poll(void);
static void spi_setup(void);
static uint8_t idle_stack[256];
static uint8_t master_thread_stack[1024];
static ATOM_TCB master_thread_tcb;

static ATOM_QUEUE uart2_tx;
static uint8_t uart2_tx_storage[64];

static ATOM_QUEUE uart2_rx;
static uint8_t uart2_rx_storage[64];

void _fault(int, int, const char*);
int u_write(int file, char *ptr, int len);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)
void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    while(1){
    }
};

static ATOM_SEM ibusy;

void tim3_setup(void);
void tim3_setup(void) {
  rcc_periph_clock_enable(RCC_TIM3);
  nvic_enable_irq(NVIC_TIM3_IRQ);
  timer_reset(TIM3);
  timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);
  timer_set_prescaler(TIM3, 1);
  timer_enable_preload(TIM3);
  timer_continuous_mode(TIM3);
  timer_set_period(TIM3, 24000);
  timer_disable_preload(TIM3);
  timer_enable_counter(TIM3);
  timer_enable_irq(TIM3, TIM_DIER_CC1IE);
}

void tim2_setup(void);
void tim2_setup(void) {
  /* Enable TIM2 clock. */
  rcc_periph_clock_enable(RCC_TIM2);

  /* Enable TIM2 interrupt. */
  nvic_enable_irq(NVIC_TIM2_IRQ);

  /* Reset TIM2 peripheral. */
  timer_reset(TIM2);

  /* Timer global mode:
   * - No divider
   * - Alignment edge
   * - Direction up
   */
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);

  /* Reset prescaler value. */
  timer_set_prescaler(TIM2, 128);

  /* Enable preload. */
  timer_enable_preload(TIM2);

  /* Continous mode. */
  timer_continuous_mode(TIM2);

  /* Period (36kHz). */
  timer_set_period(TIM2, 65535);

  /* Disable outputs. */
  timer_disable_oc_output(TIM2, TIM_OC1);
  timer_disable_oc_output(TIM2, TIM_OC2);
  timer_disable_oc_output(TIM2, TIM_OC3);
  timer_disable_oc_output(TIM2, TIM_OC4);

  /* -- OC1 configuration -- */

  /* Configure global mode of line 1. */
  timer_disable_oc_clear(TIM2, TIM_OC1);
  timer_disable_oc_preload(TIM2, TIM_OC1);
  timer_set_oc_slow_mode(TIM2, TIM_OC1);
  timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);

  /* Set the capture compare vale for OC1. */
  //	timer_set_oc_value(TIM2, TIM_OC1, 1000);

  /* ---- */

  /* ARR reload enable. */
  timer_disable_preload(TIM2);

  /* Counter enable. */
  timer_enable_counter(TIM2);

  /* Enable commutation interrupt. */
  //timer_enable_irq(TIM2, TIM_DIER_CC1IE);

  if (atomSemCreate (&ibusy, 0) != ATOM_OK) 
    fault(3);
}

void idelay(uint32_t t);
void idelay(uint32_t t){
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
    timer_set_counter(TIM2, t);
    TIM_DIER(TIM2) |= TIM_DIER_UIE;
    //timer_enable_irq(TIM2, TIM_DIER_UIE);
    atomSemGet (&ibusy, 0);
}

void xcout(unsigned char c);
void icout(unsigned char c);
void si2cout(int16_t c);
void i2cout(uint16_t c);

void xcout(unsigned char c){
    static char set[]="0123456789ABCDEF";
    char t[2]={set[(c>>4)&0x0f],set[c&0x0f]};
    u_write(1,t,2);
}

void x2cout(uint32_t c);
void x2cout(uint32_t c){
    static char set[]="0123456789ABCDEF";
    char t[8]={
        set[(c>>12)&0x0f],
        set[(c>>8)&0x0f],
        set[(c>>4)&0x0f],
        set[c&0x0f]
    };
    u_write(1,t,8);
}


void sx4cout(int32_t c);
void sx4cout(int32_t c){
  if(c & 0x80000000){
    c^=0xffffffff;
    c-=1;
    u_write(1,"-",1);
  }
  static char set[]="0123456789ABCDEF";
    char t[8]={
        set[(c>>28)&0x0f],
        set[(c>>24)&0x0f],
        set[(c>>20)&0x0f],
        set[(c>>16)&0x0f],
        set[(c>>12)&0x0f],
        set[(c>>8)&0x0f],
        set[(c>>4)&0x0f],
        set[c&0x0f]
    };
    u_write(1,t,8);
}


void x4cout(uint32_t c);
void x4cout(uint32_t c){
    static char set[]="0123456789ABCDEF";
    char t[8]={
        set[(c>>28)&0x0f],
        set[(c>>24)&0x0f],
        set[(c>>20)&0x0f],
        set[(c>>16)&0x0f],
        set[(c>>12)&0x0f],
        set[(c>>8)&0x0f],
        set[(c>>4)&0x0f],
        set[c&0x0f]
    };
    u_write(1,t,8);
}

void icout(unsigned char c){
    char t[3]="000";
    t[2]='0'+c%10;
    c/=10;
    t[1]='0'+c%10;
    c/=10;
    t[0]=c+'0';
    u_write(1,t,3);
}

void i2cout(uint16_t c){
    char t[5]="00000";
    t[4]='0'+c%10;
    c/=10;
    t[3]='0'+c%10;
    c/=10;
    t[2]='0'+c%10;
    c/=10;
    t[1]='0'+c%10;
    c/=10;
    t[0]=c+'0';
    u_write(1,t,5);
}

void si2cout(int16_t c){
  if(c & 0x8000){
    c^=0xffff;
    c-=1;
    u_write(1,"-",1);
  }
  char t[5]="00000";
  t[4]='0'+c%10;
  c/=10;
  t[3]='0'+c%10;
  c/=10;
  t[2]='0'+c%10;
  c/=10;
  t[1]='0'+c%10;
  c/=10;
  t[0]=c+'0';
  u_write(1,t,5);
}

void usart2_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART2);

        if(data=='\r' || data=='\n'){
            data='\r';
            u_write(1,(char*) &data, 1);
            data='\n';
            u_write(1,(char*) &data, 1);
        }else{
            atomQueuePut(&uart2_rx,0, (uint8_t*) &data);
        }

    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart2_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send(USART2, data);
        }else{
            USART_CR1(USART2) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

//  0 0    0  2 -2
//  1 30   1  1 -3
//  2 60   2  0 -2
//  3 90   3 -1 -1
//  4 120  2 -2  0
//  5 150  1 -3  1
//  6 180  0 -2  2
//  7 210 -1 -1  3
//  8 240 -2  0  2
//  9 270 -3  1  1
// 10 300 -2  2  0
// 11 330 -1  3 -1

#define Sm3 (1)
#define Sm2 (18)
#define Sm1 (63)
#define S0 (127)
#define S1 (191)
#define S2 (238)
#define S3 (255)

/*
uint8_t pos(int8_t v){
  switch(v){
    case -3: return 0;
    case -2: return 18;
    case -1: return 63;
    case 0: return 127;
    case 1: return 191;
    case 2: return 238;
    case 3: return 255;
  }
};*/

void pwm(uint8_t t1, uint8_t t2, uint8_t t3);
void setstep(uint8_t s){
  switch(s){
    case 0:  pwm(S0,  S2,  Sm2); break;
    case 1:  pwm(S1,  S1,  Sm3); break;
    case 2:  pwm(S2,  S0,  Sm2); break;
    case 3:  pwm(S3,  Sm1, Sm1); break;
    case 4:  pwm(S2,  Sm2, S0 ); break;
    case 5:  pwm(S1,  Sm3, S1 ); break;
    case 6:  pwm(S0,  Sm2, S2 ); break;
    case 7:  pwm(Sm1, Sm1, S3 ); break;
    case 8:  pwm(Sm2, S0,  S2 ); break;
    case 9:  pwm(Sm3, S1,  S1 ); break;
    case 10: pwm(Sm2, S2,  S0 ); break;
    case 11: pwm(Sm1, S3,  Sm1); break;
  }
};

void pwm(uint8_t t1, uint8_t t2, uint8_t t3){
  timer_set_oc_value(TIM1, TIM_OC1, t1);
  timer_set_oc_value(TIM1, TIM_OC4, t2);
  timer_set_oc_value(TIM1, TIM_OC3, t3);
};

void run_motor(){
  gpio_set(GPIOB,GPIO10);
  gpio_set(GPIOB,GPIO11);
  gpio_set(GPIOB,GPIO2);
  TIM_SR(TIM2) &= ~TIM_SR_UIF;
  timer_set_counter(TIM2, SDELAY);
  TIM_DIER(TIM2) |= TIM_DIER_UIE;
}
static void master_thread(uint32_t args __maybe_unused) {
  u_write(1,(char *)"master thread\r\n",15);
  while(1){
    uint8_t data;
    //int status = atomQueueGet(&uart2_rx, SYSTEM_TICKS_PER_SEC>>2, (void*)&data);
    int status = atomQueueGet(&uart2_rx, -1, (void*)&data);
    if(status == ATOM_OK){
#if 1
      switch (data) {
        case '1':
          x4cout(timer_get_counter(TIM1));
          u_write(1,(char *)"\r\n",2);
          break;
        case '2':
          x4cout(timer_get_counter(TIM2));
          u_write(1,(char *)"\r\n",2);
          break;
        case '3':
          x4cout(timer_get_counter(TIM3));
          u_write(1,(char *)"\r\n",2);
          break;
        case 'a':
          offset=-30;
          run_motor();
          break;
        case 's':
          offset=30;
          run_motor();
          break;
        case 'z':
          offset=-100;
          run_motor();
          break;
        case 'x':
          offset=100;
          run_motor();
          break;
        case '0':
          u_write(1,(char *)"DI\r\n",4);
          pwm(0,0,0);
          break;
      }
     
#endif
      if(abs(diff)>3){
        si2cout(diff);
        u_write(1,(char *)" ",1);
        sx4cout(gpos);
        u_write(1,(char *)"\r\n",2);
      }
    };
    //int16_t p=spi_poll();
    //gpio_set(GPIOA, GPIO4|GPIO5);
    //atomTimerDelay(SYSTEM_TICKS_PER_SEC);
  }
}

int u_write(int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        switch(file){
            case 1: //DEBUG
                atomQueuePut(&uart2_tx,0, (uint8_t*) &ptr[i]);
                break;
         }
    }
    switch(file){
        case 1:
            USART_CR1(USART2) |= USART_CR1_TXEIE;
            break;
    }
    return i;
}




#define FALLING 0
#define RISING 1

static void gpio_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);

  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

  rcc_periph_clock_enable(RCC_GPIOB);

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO0|GPIO1|GPIO2|GPIO10|GPIO11);
  gpio_set(GPIOB,GPIO0);
}

static void tim_setup(void)
{
  /* Enable TIM1 clock. */
  rcc_periph_clock_enable(RCC_TIM1);

  /* Enable GPIOA, GPIOB and Alternate Function clocks. */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_AFIO);

  /*
   * Set TIM1 channel output pins to
   * 'output alternate function push-pull'.
   */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
      GPIO_TIM1_CH1 | GPIO_TIM1_CH3 | GPIO_TIM1_CH4);

  /*
   * Set TIM1 complementary channel output pins to
   * 'output alternate function push-pull'.
   */
  /*
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
      GPIO_TIM1_CH1N | GPIO_TIM1_CH2N | GPIO_TIM1_CH3N);
      */
  /* Enable TIM1 commutation interrupt. */
  //nvic_enable_irq(NVIC_TIM1_TRG_COM_IRQ);

  /* Reset TIM1 peripheral. */
  rcc_periph_reset_pulse(RST_TIM1);

  /* Timer global mode:
   * - No divider
   * - Alignment edge
   * - Direction up
   */
  timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
      TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  /* Reset prescaler value. */
  timer_set_prescaler(TIM1, 4);

  /* Reset repetition counter value. */
  timer_set_repetition_counter(TIM1, 0);

  /* Enable preload. */
  timer_enable_preload(TIM1);

  /* Continuous mode. */
  timer_continuous_mode(TIM1);

  /* Period (32kHz). */
  timer_set_period(TIM1, 255);

  /* Configure break and deadtime. */
  /*
  timer_set_deadtime(TIM1, 10);
  timer_set_enabled_off_state_in_idle_mode(TIM1);
  timer_set_enabled_off_state_in_run_mode(TIM1);
  timer_disable_break(TIM1);
  timer_set_break_polarity_high(TIM1);
  timer_disable_break_automatic_output(TIM1);
  timer_set_break_lock(TIM1, TIM_BDTR_LOCK_OFF);
  */

  /* -- OC1 and OC1N configuration -- */

  /* Disable outputs. */
  timer_disable_oc_output(TIM1, TIM_OC1);
  timer_disable_oc_output(TIM1, TIM_OC1N);

  /* Configure global mode of line 1. */
  timer_disable_oc_clear(TIM1, TIM_OC1);
  timer_enable_oc_preload(TIM1, TIM_OC1);
  timer_set_oc_slow_mode(TIM1, TIM_OC1);
  timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);

  /* Configure OC1. */
  timer_set_oc_polarity_high(TIM1, TIM_OC1);
  timer_set_oc_idle_state_set(TIM1, TIM_OC1);

  /* Configure OC1N. */
  //timer_set_oc_polarity_high(TIM1, TIM_OC1N);
  //timer_set_oc_idle_state_set(TIM1, TIM_OC1N);

  /* Set the capture compare value for OC1. */
  timer_set_oc_value(TIM1, TIM_OC1, 100);

  /* Reenable outputs. */
  timer_enable_oc_output(TIM1, TIM_OC1);
//  timer_enable_oc_output(TIM1, TIM_OC1N);

  /* -- OC3 and OC3N configuration -- */

  /* Disable outputs. */
  timer_disable_oc_output(TIM1, TIM_OC3);
  timer_disable_oc_output(TIM1, TIM_OC3N);

  /* Configure global mode of line 3. */
  timer_disable_oc_clear(TIM1, TIM_OC3);
  timer_enable_oc_preload(TIM1, TIM_OC3);
  timer_set_oc_slow_mode(TIM1, TIM_OC3);
  timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

  /* Configure OC3. */
  timer_set_oc_polarity_high(TIM1, TIM_OC3);
  timer_set_oc_idle_state_set(TIM1, TIM_OC3);

  /* Configure OC3N. */
  timer_set_oc_polarity_high(TIM1, TIM_OC3N);
  timer_set_oc_idle_state_set(TIM1, TIM_OC3N);

  /* Set the capture compare value for OC3. */
  timer_set_oc_value(TIM1, TIM_OC3, 100);

  /* Reenable outputs. */
  timer_enable_oc_output(TIM1, TIM_OC3);
//  timer_enable_oc_output(TIM1, TIM_OC3N);

  timer_disable_oc_output(TIM1, TIM_OC4);
  timer_disable_oc_clear(TIM1, TIM_OC4);
  timer_enable_oc_preload(TIM1, TIM_OC4);
  timer_set_oc_slow_mode(TIM1, TIM_OC4);
  timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
  timer_set_oc_polarity_high(TIM1, TIM_OC4);
  timer_set_oc_idle_state_set(TIM1, TIM_OC4);
  timer_set_oc_value(TIM1, TIM_OC4, 200);
  timer_enable_oc_output(TIM1, TIM_OC4);

  /* ---- */

  /* ARR reload enable. */
  timer_enable_preload(TIM1);

  /*
   * Enable preload of complementary channel configurations and
   * update on COM event.
   */
  timer_enable_preload_complementry_enable_bits(TIM1);

  /* Enable outputs in the break subsystem. */
  timer_enable_break_main_output(TIM1);

  /* Counter enable. */
  timer_enable_counter(TIM1);

  /* Enable commutation interrupt. */
  //timer_enable_irq(TIM1, TIM_DIER_COMIE);
  pwm(0,0,0);
}


uint8_t polltoggle=0;
void tim3_isr(void) {
  if (timer_get_flag(TIM3, TIM_SR_UIF)) {
    //timer_clear_flag(TIM3, TIM_SR_UIF);
    timer_clear_flag(TIM3, 0xff);
    //TIM_SR(TIM3) &= ~TIM_SR_UIF;
    gpio_toggle(GPIOB,GPIO1);
    if(polltoggle){
      timer_set_counter(TIM3,24000-1000-1140);
      polltoggle=0;
      position=spi_poll2();

      diff=position-ppoll;
      if(diff>16384){
        if(position>16384 && ppoll<16384)
          diff=-((32768-position)+(ppoll));
        else diff=65535;
      }else if(diff<-16384){
        if(position<16384 && ppoll>16384)
          diff=(32768-ppoll)+(position);
        else diff=65535;
      }
      gpos+=diff;
      ppoll=position;
    }else{
      timer_set_counter(TIM3,1000);
      polltoggle=1;
      spi_poll1();
    }
  }
}


void tim2_isr(void) {
    atomIntEnter();
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
      TIM_SR(TIM2) &= ~TIM_SR_UIF;
      if(offset!=0){
        if(offset<0){
          offset++;
          step++;
          if(step>11) step=0;
        };
        if(offset>0){
          offset--;
          step--;
          if(step<0) step=11;
        }
        i2cout(step*30);
        setstep(step);
        timer_set_counter(TIM2, SDELAY);
        u_write(1,(char *)"=\r\n",3);
        TIM_DIER(TIM2) |= TIM_DIER_UIE;
      }else{
        pwm(0,0,0);
        gpio_clear(GPIOB,GPIO2);
        gpio_clear(GPIOB,GPIO10);
        gpio_clear(GPIOB,GPIO11);
        TIM_DIER(TIM2) &= ~TIM_DIER_UIE; //disable interrupt
      }
        //timer_clear_flag(TIM2, TIM_SR_UIF);
        //TIM_SR(TIM2) &= ~TIM_SR_UIF;
        //timer_disable_irq(TIM2, TIM_DIER_UIE);
//        TIM_DIER(TIM2) &= ~TIM_DIER_UIE; //disable interrupt
//        atomSemPut (&ibusy);
    }
    atomIntExit(0);
}

int main(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    //rcc_clock_setup_in_hse_8mhz_out_24mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART3);

    /*
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO4|GPIO5);
    gpio_clear(GPIOA, GPIO4);
    gpio_set(GPIOA, GPIO5);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO15);
    */

    usart_setup();
    spi_setup();

    cm_mask_interrupts(true);
    systick_set_frequency(SYSTEM_TICKS_PER_SEC, 48000000);
    systick_interrupt_enable();
    systick_counter_enable();

    nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

    /*
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO6|GPIO7|GPIO8|GPIO9);
    gpio_set(GPIOC, GPIO6);
    gpio_set(GPIOC, GPIO7);
    gpio_clear(GPIOC, GPIO8);
    gpio_toggle(GPIOC, GPIO9);
    */

     

    /*
    gpio_set(GPIOA, GPIO15);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO8|GPIO9);
            */

    /*
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
    gpio_clear(GPIOA, GPIO8);
    */

    /* Button pin */

    gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO1);
    gpio_set(GPIOC, GPIO1);


    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX);

    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');
    usart_send_blocking(USART2, 'U');
    usart_send_blocking(USART2, '2');
    usart_send_blocking(USART2, '\r');
    usart_send_blocking(USART2, '\n');


    tim2_setup();
    tim3_setup();

    if(atomOSInit(idle_stack, sizeof(idle_stack), FALSE) != ATOM_OK) 
        fault(1);

    if (atomQueueCreate (&uart2_rx, uart2_rx_storage, sizeof(uint8_t), sizeof(uart2_rx_storage)) != ATOM_OK) 
        fault(2);
    if (atomQueueCreate (&uart2_tx, uart2_tx_storage, sizeof(uint8_t), sizeof(uart2_tx_storage)) != ATOM_OK) 
        fault(3);
    u_write(1,(char *)"USART2 ready\r\n",14);

    gpio_setup();
    tim_setup();

    atomThreadCreate(&master_thread_tcb, 10, master_thread, 0,
            master_thread_stack, sizeof(master_thread_stack), TRUE);

    atomOSStart();
    fault(255);
}


void spi_poll1(){
  //gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7 );
  gpio_clear(GPIOA, GPIO4);
  spi_send(SPI1, 0x8021);
  spi_read(SPI1);
}
int spi_poll2(){
  /*
  gpio_clear(GPIOA, GPIO4);
  spi_send(SPI1, 0x8021);
  spi_read(SPI1);
  atomTimerDelay(1);
*/
  spi_send(SPI1, 0xffff);
  uint16_t res=spi_read(SPI1)&0x7fff;
  spi_send(SPI1, 0xffff);
  spi_read(SPI1);
  gpio_set(GPIOA, GPIO4);
  return res;
}

static void spi_setup(void) {
  rcc_periph_clock_enable(RCC_SPI1);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7 );
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4 );
  gpio_set(GPIOA, GPIO4);

/*
PA4  SPI1_NSS
PA5  SPI1_SCK
PA6 SPI1_MISO
PA7 SPI1_MOSI
*/

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI1);

  SPI1_I2SCFGR = 0; //disable i2s
  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
      SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  spi_enable(SPI1);
}

