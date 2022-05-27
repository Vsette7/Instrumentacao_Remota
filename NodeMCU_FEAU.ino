// BT5 - Dispositivo 1
// BT3 - Dispositivo 2
// BT6 - Dispositivo 3
// BT7 - Dispositivo 4
// BT2 - Dispositivo 5
// BT1 - Dispositivo 6

#include <ESP8266WiFi.h> // Importa a Biblioteca ESP8266WiFi
#include <PubSubClient.h> // Importa a Biblioteca PubSubClient
//#include <SHT1x.h>
#include <Wire.h>
#include "accel.h"

//defines - mapeamento de pinos do NodeMCU - ESP-07(16 = D16)
#define D0    16
#define D1    5
#define D2    4
#define D3    0
#define D4    2
#define D5    14
#define D6    12
#define D7    13
#define D8    15
#define D9    3
#define D10   1

// ADXL345 I2C address is 0x53(83)
#define Addr 0x53

const uint8_t scl = 14; //D5
const uint8_t sda = 12; //D6

extern int xAccl;
extern int yAccl;
extern int zAccl;

//defines:
//defines de id mqtt e tópicos para publicação e subscribe
#define TOPICO_SUBSCRIBE "feau/acelerometro"     //tópico MQTT de escuta. 
#define topicLive "feau/sensor/live"
                                   //ATENÇÃO: deve ser o mesmo tópico de publish utilizado na Raspberry PI!!!
                                                 
#define ID_MQTT "sensor"     //id mqtt (para identificação de sessão)
                                        //IMPORTANTE: este deve ser único no broker (ou seja, 
                                        //            se um client MQTT tentar entrar com o mesmo 
                                        //            id de outro já conectado ao broker, o broker 
                                        //            irá fechar a conexão de um deles).
                               

// WIFI

//const char* SSID = "IOT"; // SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "iotalpha"; // Senha da rede WI-FI que deseja se conectar


const char* SSID = "UnivapWifi"; // SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "universidade"; // Senha da rede WI-FI que deseja se conectar

char live_status = 0;

long lastMsg = 0;
 
// MQTT
const char* BROKER_MQTT = "broker.hivemq.com"; //URL do broker MQTT que se deseja utilizar
//const char* BROKER_MQTT = "192.168.1.1"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883;//1881; // Porta do Broker MQTT

//Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient
char EstadoSaida = '0';  //variável que armazena o estado atual da saída
char CmdNodeMCU[3] = {'B','T', '1'};   //ATENÇÃO: troque o "BBB" por um dos códigos enviados pela Raspberry PI ("BT1", "BT2", "BT3", "BT4" ou "BT5"),
                                       //         de modo que cada NodeMCU "espere" apenas por um deles
 
//Prototypes
void initSerial();
void initWiFi();
void initMQTT();
void reconectWiFi(); 
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void VerificaConexoesWiFIEMQTT(void);
void InitOutput(void);

/* 
 *  Implementações das funções
 */
void setup() 
{
    //inicializações:
    InitOutput();
    initSerial();
    // Initialise I2C communication as MASTER
    Wire.begin(sda, scl);
    initWiFi();
    initMQTT();
}
 
//Função: inicializa comunicação serial com baudrate 115200 (para fins de monitorar no terminal serial 
//        o que está acontecendo.
//Parâmetros: nenhum
//Retorno: nenhum
void initSerial() 
{
    Serial.begin(115200);
}

//Função: inicializa e conecta-se na rede WI-FI desejada
//Parâmetros: nenhum
//Retorno: nenhum
void initWiFi() 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
    
    reconectWiFi();
}
 
//Função: inicializa parâmetros de conexão MQTT(endereço do 
//        broker, porta e seta função de callback)
//Parâmetros: nenhum
//Retorno: nenhum
void initMQTT() 
{
    MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
 
//Função: função de callback 
//esta função é chamada toda vez que uma informação de 
//um dos tópicos subescritos chega)
//Parâmetros: nenhum
//Retorno: nenhum
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    char MsgRecebida[3];
    
    //se a mensagem não tem três caracteres (ou seja, não é "BT1", "BT2", "BT3", "BT4" ou "BT5"), 
    //automaticamente ela é inválida. Nesse caso, nada mais é feito com a mensagem recebida.
    if (length != 3)
      return;

    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
      MsgRecebida[i] = (char)payload[i];
      Serial.print(MsgRecebida[i]);
    }
       
   
    //avalia se a mensagem é para este NodeMCU
    if (memcmp(MsgRecebida,CmdNodeMCU,3) == 0)
    {
      digitalWrite(D0,HIGH);
      EstadoSaida = '1';
      Serial.println("LIGADO!");   
    }else
    {
      digitalWrite(D0,LOW);
      EstadoSaida = '0';
      Serial.println("DESLIGADO!");    
    }    
}
 
//Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
//        em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
//Parâmetros: nenhum
//Retorno: nenhum
void reconnectMQTT() 
{
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE);
            MQTT.subscribe(topicLive);
            digitalWrite(D1, HIGH); 
        } 
        else 
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            digitalWrite(D1, LOW);
            delay(2000);
        }
    }
}
 
//Função: reconecta-se ao WiFi
//Parâmetros: nenhum
//Retorno: nenhum
void reconectWiFi() 
{
    //se já está conectado a rede WI-FI, nada é feito. 
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
        
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
    
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
  
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}

//Função: verifica o estado das conexões WiFI e ao broker MQTT. 
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFIEMQTT(void)
{
    if (!MQTT.connected()) 
        reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
    
     reconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}

//Função: inicializa o output em nível lógico baixo
//Parâmetros: nenhum
//Retorno: nenhum
void InitOutput(void)
{
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    digitalWrite(D0, LOW);
    digitalWrite(D1, LOW);         
    EstadoSaida = '0';
}


//programa principal
void loop() 
{     
    //garante funcionamento das conexões WiFi e ao broker MQTT
    VerificaConexoesWiFIEMQTT();

  long now = millis();
  
  if (now - lastMsg > 3000) {
    lastMsg = now;

    live_status = !live_status;

    if(yAccl > 100) MQTT.publish(TOPICO_SUBSCRIBE, "direita");else
    if(yAccl < -300) MQTT.publish(TOPICO_SUBSCRIBE, "esquerda");else
    MQTT.publish(TOPICO_SUBSCRIBE, "nivelado");
    //MQTT.publish(topicLive, live_status ? "1" : "0");
    
  }
    delay(1000);
    
    //keep-alive da comunicação com broker MQTT
    MQTT.loop();
    handleroot();
}
