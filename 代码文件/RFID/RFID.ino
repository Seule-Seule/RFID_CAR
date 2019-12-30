#include <SPI.h>
#include <MFRC522.h>
 
#define SS_PIN 10
#define RST_PIN 9
#define Out_Res 4

MFRC522 rfid(SS_PIN, RST_PIN); //实例化类
 
// 初始化数组用于存储读取到的NUID 
byte nuidPICC[4];
byte myPICC[4] = {144,87,71,167};
byte samenum = 0;
bool  flag = 0;  // 未授权


void setup() { 
  pinMode(Out_Res, OUTPUT);
  Serial.begin(9600);
  SPI.begin(); // 初始化SPI总线
  rfid.PCD_Init(); // 初始化 MFRC522 
}
 
void loop() {

  //flag  = 0;
  samenum = 0;
  // 找卡
  if ( ! rfid.PICC_IsNewCardPresent())
    return;
 
  // 验证NUID是否可读
  if ( ! rfid.PICC_ReadCardSerial())
    return;
 
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
 
  // 检查是否MIFARE卡类型
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println("不支持读取此卡类型");
    return;
  }
  
  // 将NUID保存到nuidPICC数组
  for (byte i = 0; i < 4; i++) {
    if (myPICC[i] == rfid.uid.uidByte[i]){
      samenum++;
    }
    Serial.println(rfid.uid.uidByte[i]);
    Serial.println(myPICC[i]);
  }   
  Serial.print("十六进制UID：");
  printHex(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  if (samenum == 4){
    flag = !flag;
  }
//  Serial.print("十进制UID：");
//  printDec(rfid.uid.uidByte, rfid.uid.size);
//  Serial.println();
//  
  // 使放置在读卡区的IC卡进入休眠状态，不再重复读卡
  rfid.PICC_HaltA();
 
  // 停止读卡模块编码
  rfid.PCD_StopCrypto1();
  if (flag == 1 && samenum == 4){
    digitalWrite(Out_Res, HIGH);  // 授权成功，传递成功信号
    Serial.print("HIGH"); 
  }
  else {
    digitalWrite(Out_Res, LOW);
    Serial.print("LOW");
  }
}
 
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : "");
    Serial.print(buffer[i], HEX);
  }
}
 
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : "");
    Serial.print(buffer[i], DEC);
  }
}
