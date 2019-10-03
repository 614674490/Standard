#ifndef _Init_Music_h
#define _Init_Music_h

/*                    蜂鸣器控制参数                           */
extern int32_t auto_counter;		//用于准确延时的完成某事件
/**
 * 占空比
 */
#define L1	262
#define L1U	277
#define L2	294
#define L2U	311
#define L3	330
#define L4	349
#define L4U	370
#define L5	392
#define L5U	415
#define L6	440
#define L6U	466
#define L7	494

#define M1	523
#define M1U	554
#define M2	587
#define M2U	622
#define M3	659
#define M4	698
#define M4U	740
#define M5	784
#define M5U	831
#define M6	880
#define M6U	932
#define M7	988

#define H1	1046
#define H1U	1109
#define H2	1175
#define H2U	1245
#define H3	1318
#define H4	1397
#define H4U	1480
#define H5	1568
#define H5U	1661
#define H6	1760
#define H6U	1865
#define H7	1976

/**
 *重装载值 占空比
 */
typedef struct {
	uint16_t note;
	uint16_t time;
}MusicNote;

void PLAY(uint16_t note,uint16_t  time);
void set_random_music(void);
void playSuperMarioMusic(void);
void playqinghauciMusic(void);
void playNufangMusic(void);
void playCanghaixiaoMusic(void);
void random_play(void);
void playrobotcatMusic(void);
extern int Init_End_Flag;   
#endif


