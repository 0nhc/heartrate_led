// 导入心率传感器相关库的头文件
#include <DFRobot_MAX30102.h>
// 导入LED灯带相关库的头文件
#include <FastLED.h>

// 定义LED灯带的接口
#define LED_PIN 6
// 定义LED灯带上的灯珠数量
#define NUM_LEDS 150
// 定义FIFO队列的长度
#define FIFO_LEN 75
// 定义均值滤波器的缓存列表长度
#define FILTER_LEN 20

// 定义类：均值滤波器
class Filter
{
  public:
    // 均值滤波器的构造函数
    Filter()
    {
      // 将均值滤波器的缓存列表内的数据全部初始化为0
      for(int i=0; i<FILTER_LEN; i++)
      {
        cache[i] = 0;
      }
    };

    // 均值滤波器的缓存列表，一共包含FILTER_LEN个int型数据
    int cache[FILTER_LEN];

    // 缓存列表更新函数。将新的数据放到缓存列表的尾部，然后将缓存列表中的第一个数据删除，最终得到新的缓存列表。
    void update(int data)
    {
      for(int i=1; i<FILTER_LEN-1; i++)
      {
        cache[i-1] = cache[i];
      }
      cache[FILTER_LEN-1] = data;
    }

    // 计算缓存列表中所有数据的平均值
    int average()
    {
      unsigned long sum = 0;
      for(int i=0; i<FILTER_LEN; i++)
      {
        sum += cache[i];
      }
      return (int)(sum/FILTER_LEN);
    }
};

// 定义类：用于储存心率传感器数据的FIFO队列
class FIFO
{
  public:
    // FIFO队列的构造函数
    FIFO()
    {
      // 将FIFO队列内的所有数据初始化为0
      for(int i=0; i<FIFO_LEN; i++)
      {
        cache[i] = 0;
      }
    };

    //FIFO队列，一共包含FIFO_LEN个unsigned long型数据
    unsigned long cache[FIFO_LEN];

    // FIFO队列更新函数。将新的数据放到FIFO队列的尾部，然后将FIFO队列中的第一个数据删除，最终得到新的FIFO队列。
    void update(unsigned long data)
    {
      for(int i=1; i<FILTER_LEN; i++)
      {
        cache[i-1] = cache[i];
      }
      cache[FILTER_LEN-1] = data;
    }

    // 获取FIFO队列内数据的最大值
    unsigned long getMax()
    {
      unsigned long max = 0;
      for(int i=0; i<FILTER_LEN; i++)
      {
        if(cache[i] > max)
        {
          max = cache[i];
        }
      }
      return max;
    }

    // 获取FIFO队列内数据的最小值
    unsigned long getMin()
    {
      unsigned long min = 1000000;
      for(int i=0; i<FILTER_LEN; i++)
      {
        if(cache[i] < min)
        {
          min = cache[i];
        }
      }
      return min;
    }
};

// 生成DFRobot_MAX30102类型的对象particleSensor，用于读取心率传感器数据
DFRobot_MAX30102 particleSensor;
// 定义灯带的亮度
int brightness = 0;
// 生成Filter类型的对象filter，用于对灯带亮度进行均值滤波
Filter filter;
// 生成FIFO类型的对象ir_fifo，用于储存心率传感器的ir数据
FIFO ir_fifo;
// 生成FIFO类型的对象r_fifo，用于储存心率传感器的r数据
FIFO r_fifo;

// 生成CRGB类型的对象列表leds，长度为NUM_LEDS
CRGB leds[NUM_LEDS];
// 亮度系数，手动调整，值越大越亮
float brightness_factor = 15.0;

// 对心率传感器的读数映射到灯带亮度。
int heartrate_map(unsigned long min, unsigned long max, unsigned long value)
{
  // 根据当前值value在最小值min和最大值max之间的位置，将value映射到0-255的范围
  return (int)(((double)value-(double)min)*(255.0/((double)max-(double)min)));
}

// 点亮灯带函数。brightness范围0-255，值越大灯带越亮
void setBrightness(int brightness)
{
  for(int i=0; i<NUM_LEDS; i++)
  {
    leds[i] = CRGB (brightness, brightness*69/255, 0);
  }
  FastLED.show();
}

// 程序初始化
void setup()
{
  // 开启串口通讯
  Serial.begin(9600);

  // 初始化心率传感器
  while (!particleSensor.begin())
  {
    Serial.println("MAX30102 was not found");
    delay(1000);
  }
  particleSensor.sensorConfiguration(/*ledBrightness=*/0x1F, /*sampleAverage=*/SAMPLEAVG_4, \
                                  /*ledMode=*/MODE_MULTILED, /*sampleRate=*/SAMPLERATE_400, \
                                  /*pulseWidth=*/PULSEWIDTH_411, /*adcRange=*/ADCRANGE_4096);

  // 初始化灯带                                  
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}

// 程序主循环
void loop()
{
  // 从心率传感器读取ir和r数据
  unsigned long ir = particleSensor.getIR();
  unsigned long r = particleSensor.getRed();
  // 更新ir和r的FIFO队列
  ir_fifo.update(ir);
  r_fifo.update(r);

  // 根据ir和r的FIFO队列中的最大值与最小值，把ir和i映射到0-255的范围
  int ir_brightness = heartrate_map(ir_fifo.getMin(), ir_fifo.getMax(), ir);
  int r_brightness = heartrate_map(r_fifo.getMin(), r_fifo.getMax(), r);

  // 如果读数正常（传感器前确实存在物体）
  if(ir > 10000 && r > 10000)
  {
    // 根据ir和r映射得到的灯带亮度取平均值，获得结合ir和r两个数据之后的灯带亮度
    brightness = (ir_brightness + r_brightness) / 2.0;
  }
  // 如果读数不正常（传感器前没有物体）
  else
  {
    // 把灯带亮度设置为0
    brightness = 0;
  }

  // 更新均值滤波器中的灯带亮度数据
  filter.update(brightness);
  // 对灯带亮度进行均值滤波
  brightness = filter.average();
  // 均值滤波得到的亮度可能会变小，这取决于FILTER_LEN的大小。于是把灯带亮度乘以灯带亮度系数复原到正常亮度
  brightness = (int) brightness*brightness_factor;

  // 在串口监视器中打印灯带亮度
  Serial.println(brightness);
  // 根据计算得到的灯带亮度点亮灯带
  setBrightness(brightness);
}
