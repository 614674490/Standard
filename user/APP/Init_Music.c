#include "include.h"
int32_t auto_counter=0;
u32 music_random=0;  

/** 
 * 音调参数 小星星 沧海一声笑 怒放 青花瓷 机器猫
 */
MusicNote SuperMario[] = {
	{M1, 100}, {0, 25}, 
	{M1, 250}, {0, 25}, 
	{M5, 100}, {0, 25}, 
	{M5, 100}, {0, 25},  
	{M6, 250}, {0, 25},
	{M6, 250}, {0, 25},
	{M5, 250}, {0, 25},
	{0, 150},
	{M4, 250}, {0, 25},
	{M4, 250}, {0, 25},
	{M3, 100}, {0, 25}, 
	{M3, 250}, {0, 25}, 
	{M2, 100}, {0, 25}, 
	{M2, 100}, {0, 25}, 
	{M1, 100}, {0, 25}, 
	{0, 150},
};
 
 MusicNote qinghauci[] = {
	{M5, 100}, {0, 25}, 
	{M5, 100}, {0, 25}, 
	{M3, 100}, {0, 25}, 
	{M2, 100}, {0, 25},  
	{M3, 100}, {0, 25},
	{L6, 100}, {0, 25},
	{0, 150},
	{M2, 100}, {0, 25},
	{M3, 100}, {0, 25},
	{M5, 100}, {0, 25}, 
	{M3, 100}, {0, 25}, 
	{M2, 250}, {0, 25}, 
	{0, 150},
};
 
 MusicNote Nufang[] = {
	{M5, 100}, {0, 25}, 
	{M5, 120}, {0, 25}, 
	{M6, 100}, {0, 25}, 
	{H5, 30}, {0, 25},  
	{H3, 30}, {0, 25},
	{H1, 80}, {0, 25},
	{H3, 30},{0, 25},
	{H3, 30}, {0, 25},
	{0, 10},
	
	{H2, 100}, {0, 25},
	{H3, 100}, {0, 25}, 
	{H5, 130}, {0, 25}, 
	{H3, 120}, {0, 25},
 	{H2, 100}, {0, 25},
	{H1, 100}, {0, 25}, 
	{H1, 100}, {0, 25}, 
	{H2, 100}, {0, 25},
 	{H1, 100}, {0, 25},
	{H1, 20}, {0, 25}, 
	{0, 150},
	
	{H2, 100}, {0, 25},
	{H3, 100}, {0, 25}, 
	{H5, 130}, {0, 25}, 
	{H3, 120}, {0, 25},
 	{H2, 100}, {0, 25},
	{H1, 100}, {0, 25}, 
	{H2, 100}, {0, 25}, 
	{H2, 100}, {0, 25},
 	{H2, 100}, {0, 25},
	{M6, 20}, {0, 25}, 
	{0, 150},
		
	{M6, 100}, {0, 25},
	{H1, 100}, {0, 25}, 
	{H2, 100}, {0, 25}, 
	{H2, 100}, {0, 25},
 	{H2, 150}, {0, 25},
	{H1, 30}, {0, 25}, 
	{M6, 100}, {0, 25}, 
	{H1, 100}, {0, 25},
 	{H1, 20}, {0, 25},
	{0, 150},
};


MusicNote Canghaixiao[] = {
	{M6, 100}, {0, 25},
	{M5, 100}, {0, 25}, 
	{M3, 150}, {0, 25}, 
	{M2, 100}, {0, 10},  
	{M1, 50}, {0, 25},
	{0, 300},
	{M3, 200}, {0, 10},
	{M2, 100}, {0, 10},
	{L1, 100}, {0, 25}, 
	{L6, 100}, {0, 10}, 
	{L5, 50}, {0, 25}, 
	{0, 50},

  {L5, 100}, {0, 25},
	{L6, 100}, {0, 10}, 
	{L5, 100}, {0, 25},
	{L6, 50}, {0, 10}, 
	{M1, 150}, {0, 25}, 
	{M2, 100}, {0, 10},  
	{M3, 100}, {0, 25},
	{M5, 50}, {0, 5},
	{M6, 100},{0, 10},
	{M5, 100}, {0, 10},
	{M3, 100}, {0, 10},
	{M2, 100}, {0, 10}, 
	{M1, 50}, {0, 25}, 
	{0, 50},
	
};


MusicNote robotcat[] = {
	{L5, 500}, {0, 800},
	{M1, 500}, {0, 800}, 
	{M1, 500}, {0, 800}, 
	{M3, 500}, {0, 800}, 
	{M6, 500}, {0, 800},
	{M3, 500}, {0, 800},
	{M5, 500}, {0, 800},
	{0, 400},
	
	{M5, 500}, {0, 900}, 
	{M6, 500}, {0, 900}, 
	{M5, 500}, {0, 900}, 
  {M3, 500}, {0, 900},
	{M4, 500}, {0, 900}, 
	{M3, 500}, {0, 900},
	{M2, 500}, {0, 900}, 
	{0, 400},
	
	{L6, 500}, {0, 800}, 
	{M2, 500}, {0, 800},
	{M2, 500}, {0, 800}, 
	{M4, 500}, {0, 800}, 
	
	{M7, 500}, {0, 800}, 
	{M7, 500}, {0, 800},
	{M6, 500}, {0, 800}, 
	{M5, 500}, {0, 800}, 
	
	{M4, 500}, {0, 900}, 
	{M4, 500}, {0, 900},
	{M6, 800}, {0, 900},
	{M7, 800}, {0, 900}, 
	{M5, 500}, {0, 900}, 
	{M1, 500}, {0, 900},
	{M2, 500}, {0, 900}, 
	{M2, 500}, {0, 900}, 
	{0, 500},
	
};

void set_random_music(void)
{
	music_random=set_random(1,5);
}
void playSuperMarioMusic(void){
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
	for(int i = 0; i <  sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
 
}

void playqinghauciMusic(void){

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
	for(int i = 0; i < sizeof(qinghauci) / sizeof(MusicNote); i++){
			PLAY(qinghauci[i].note, qinghauci[i].time);
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
 
}
void playNufangMusic(void){

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
	for(int i = 0; i < sizeof(Nufang) / sizeof(MusicNote); i++){
			PLAY(Nufang[i].note, Nufang[i].time);
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
 
}

void playCanghaixiaoMusic(void){

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
	for(int i = 0; i < sizeof(Canghaixiao) / sizeof(MusicNote); i++){
			PLAY(Canghaixiao[i].note, Canghaixiao[i].time);
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
 
}

void playrobotcatMusic(void){

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	
	for(int i = 0; i < sizeof(robotcat) / sizeof(MusicNote); i++){
			PLAY(robotcat[i].note, robotcat[i].time);
	}
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
 
}

void random_play(void)
{
	switch(music_random)
	{
		case 1: playSuperMarioMusic();break;
		case 2: playqinghauciMusic();break;
		case 3: playNufangMusic();break;
		case 4: playCanghaixiaoMusic();break;
		case 5:playrobotcatMusic();    break;
	}
}
/**
 * 播放音乐
 */
void PLAY(uint16_t note,uint16_t  time)
{ 
 
	while(auto_counter!=0)
	{
		vTaskDelay(1);
	}
	if(note == 0){ 
		__HAL_TIM_SET_AUTORELOAD(&htim12, 0);
	}else { 
		__HAL_TIM_SET_AUTORELOAD(&htim12, 1000000 / note); 
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 500000 / note); 
	}
	auto_counter=time;
}



