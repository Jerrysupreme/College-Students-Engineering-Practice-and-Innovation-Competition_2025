int UART4_IRQHandler(void)
{	
	static int Usart_Receive;
	static int last_x,last_y;
	static uint8_t isGotFrameHeader = 0;
	static uint8_t frameHeaderCount = 0;
	static uint8_t dataCount = 0;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received //ÅÐ¶ÏÊÇ·ñ½ÓÊÕµ½Êý¾Ý
	{

		Usart_Receive = USART_ReceiveData(UART4);
		
		if(isGotFrameHeader == 0)
		{
			if(Usart_Receive == FRAME_HEADER_U4)
			{
				frameHeaderCount++;
				if(frameHeaderCount == 2) //½ÓÊÜÁËÁ½´ÎÖ¡Í·
				{
					isGotFrameHeader = 1;
					frameHeaderCount = 0;
					dataCount = 0;
				}
			} 
			else
			{
				isGotFrameHeader = 0;
				frameHeaderCount = 0;
				dataCount = 0;
			}
		}
		else
		{
			USART4_RX_BUF[dataCount] = Usart_Receive;
			
			if(dataCount == 5)
			{
			last_x = Vision_X_Current;
			last_y = Vision_Y_Current;
			
			Vision_X_Current = USART4_RX_BUF[0]*256 + USART4_RX_BUF[1];
			Vision_Y_Current = USART4_RX_BUF[2]*256 + USART4_RX_BUF[3];
		  Vision_Flag = USART4_RX_BUF[4]*256 + USART4_RX_BUF[5];
				
				
			if(Vision_X_Current >800) Vision_X_Current = last_x;
			if(Vision_Y_Current >420) Vision_Y_Current = last_y;
			
				
		  
//      for(int i = 0;i<=5;i++)	 
//      {
//				printf("%x",USART4_RX_BUF[i]);
//			}				
//			printf("\n");
			if(Vision_Flag == 1) //ÊÓÒ°ÖÐÃ»Çò
				{
					if(step == 2||step == 3||step == 4)
					{
						step = 0; //ÖØÐÂ¿ªÊ¼ÕÒÇò
						Seek_Ball = 1;
						Seek_Save = 0;
					}
				}
			else if(Vision_Flag == 2)
			{
				if(step == 7)
				{
					Seek_Save = 2; //µ½´ï°²È«Çø
				}
			}	
            isGotFrameHeader = 0;
			frameHeaderCount = 0;
			}
			else
			{
				dataCount++;
			}	
		}
	}
			USART_ClearITPendingBit(UART4, USART_IT_RXNE);		//Çå³ý±êÖ¾Î»
  return 0;	
}