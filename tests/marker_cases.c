/* Included after motion_cases.c: actual encoder conversion + App display scheduling. */
static void marker_request(void)
{
  static const uint8_t request[6]={0x55,0x50,1,0xFF,0xFF,0xFF};
  uint8_t i;
  for(i=0;i<6;++i)
  { RxBuffer[0]=request[i]; HAL_UART_RxCpltCallback(&huart2); }
}

static int marker_value(const char *prefix)
{
  uint16_t i,j,k;
  int value;
  for(i=0;i<host_screen_tx_len;++i)
  {
    j=0;
    while(prefix[j] && i+j<host_screen_tx_len && host_screen_tx[i+j]==(uint8_t)prefix[j])++j;
    if(prefix[j])continue;
    value=0; k=(uint16_t)(i+j);
    while(k<host_screen_tx_len && host_screen_tx[k]>='0' && host_screen_tx[k]<='9')
      value=value*10+host_screen_tx[k++]-'0';
    return value;
  }
  return -1;
}

__declspec(dllexport) int run_marker_case(int which)
{
  float x=999,y=999;
  uint16_t px,py;
  uint32_t t0,count;
  motion_reset(); HMI_Init();
  if(which==0 || which==1)
  {
    s_pos_count[0]=which==1?1:0; s_pos_count[1]=0;
    return !corexy_get_actual_pos_fresh(&x,&y,300) && x==999 && y==999;
  }
  if(which==2)
  {
    position_packet(1,-4915); position_packet(2,-1638);
    return corexy_get_actual_pos_fresh(&x,&y,300) &&
      host_fabsf(x-2)<0.001f && host_fabsf(y-1)<0.001f;
  }
  if(which==3)
  {
    tick+=300;
    if(!corexy_get_actual_pos_fresh(&x,&y,300))return 0;
    ++tick;position_packet(1,0);
    return !corexy_get_actual_pos_fresh(&x,&y,300);
  }
  if(which==4)
  {
    tick=0xfffffff0u;position_packet(1,0);position_packet(2,0);
    tick=0x11cu;
    if(!corexy_get_actual_pos_fresh(&x,&y,300))return 0;
    ++tick;return !corexy_get_actual_pos_fresh(&x,&y,300);
  }
  if(which==5)
  {
    if(!launch_target(100,80))return 0;
    marker_request();App_Poll();
    if(!corexy_get_actual_pos_fresh(&x,&y,300) || !HMI_MapActualPosition(x,y,161,144,&px,&py))return 0;
    return host_screen_tx_count==1 && marker_value("page0.gok.val=")==1 &&
      marker_value("page0.gpx.val=")==px && marker_value("page0.gpy.val=")==py &&
      marker_value("page0.gpx.val=")<40 && Motion_IsBusy();
  }
  if(which==6)
  {
    automatic=0; s_pos_tick[1]=tick-301;
    current_packet(1,120);current_packet(2,120);
    marker_request();HMI_Poll();HMI_PublishPosition();
    return host_screen_tx_count==1 && marker_value("page0.gok.val=")==0 && !starts;
  }
  if(which==7 || which==8)
  {
    if(!launch_target(100,80))return 0;
    Motion_SubmitXY(20,20,20,"TEST");
    if(which==7)bluetooth_receive("S\r");
    else corexy_fault_stop(7);
    marker_request();App_Poll();
    /* S discards the request; the next UI timer asks again. */
    HAL_Delay(100);marker_request();App_Poll();
    return !Motion_IsBusy() && !s_has_pending && !corexy_is_calibrated() &&
      marker_value("page0.gok.val=")==0;
  }
  if(which==9)
  {
    Motion_SubmitXY(20,15,20,"TEST");t0=tick;
    while(tick-t0<6000 && Motion_IsBusy())
    {
      marker_request();App_Poll();HAL_Delay(10);
    }
    if(Motion_IsBusy() || !actual_near(20,15) || host_path_error>0.5f || host_screen_tx_count<10)return 0;
    HAL_Delay(100);marker_request();App_Poll();
    if(!corexy_get_actual_pos_fresh(&x,&y,300) || !HMI_MapActualPosition(x,y,161,144,&px,&py))return 0;
    if(marker_value("page0.gok.val=")!=1 || marker_value("page0.gpx.val=")!=px ||
       marker_value("page0.gpy.val=")!=py)return 0;
    count=host_screen_tx_count;
    HAL_Delay(100);App_Poll();HAL_Delay(100);App_Poll();
    return host_screen_tx_count==count;
  }
  return 0;
}
