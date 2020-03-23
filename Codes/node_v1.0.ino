#include <ESP8266WiFi.h>
#include <PubSubClient.h> // Importa a Biblioteca PubSubClient
#include <SoftwareSerial.h>                                  //-------------------tirar

// ---- SoftwareSerial -----
#define rxPin 4                                            // D2-------------------tirar
#define txPin 5                                            // D1-------------------tirar
SoftwareSerial Serial_ArdEsp(rxPin, txPin);

// --------- Wifi ----------
const char* ssid =      "SSID";
const char* password =  "SENHA";

// ----- Topicos Mqtt ------
#define SUB_TOPIC "happy/velocities"
#define PUB_TOPIC "happy/odometry"

// ------- Variables -------
WiFiClient espClient;
PubSubClient client(espClient);
const char* mqtt_server = "192.168.1.10";
int broker_port = 1883;
const char* mqtt_id = "ESP";

// ----- Functions -------
void initWiFi() {
  delay(10);
  Serial.println("------Conexao WI-FI------");//-------------------tirar
  Serial.print("Conectando-se na rede: ");    //-------------------tirar
  Serial.println(ssid);                       //-------------------tirar
  Serial.println("Aguarde");                  //-------------------tirar

  reconectWiFi();
}
void initMQTT() {
  client.setServer(mqtt_server, broker_port);//informa qual broker e porta deve ser conectado
  client.setCallback(callback);              //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
  client.subscribe(SUB_TOPIC);
}
void verifyConections() {
  if (!client.connected())
    reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita

  reconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}
void reconectWiFi() {
  if (WiFi.status() == WL_CONNECTED)
    return;
  WiFi.begin(ssid, password); // Conecta na rede WI-FI
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");                           //-------------------tirar
  }
  Serial.println();                              //-------------------tirar
  Serial.print("Conectado com sucesso na rede ");//-------------------tirar
  Serial.println(ssid);                          //-------------------tirar
  Serial.print("IP obtido: ");                   //-------------------tirar
  Serial.println(WiFi.localIP());                //-------------------tirar
}
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");    //-------------------tirar
    Serial.println(mqtt_server);                                //-------------------tirar
    if (client.connect(mqtt_id)) {
      Serial.println("Conectado com sucesso ao broker MQTT!");//-------------------tirar
      client.subscribe(SUB_TOPIC);
    }
    else {
      Serial.println("Falha ao reconectar no broker.");        //-------------------tirar
      Serial.println("Havera nova tentatica de conexao em 2s");//-------------------tirar
      delay(2000);
    }
  }
}

// - Debug:
#define DEBUG false

// - Máquina de estados principal:
int state = 0;

// - Máquina de estados recepção de chars:
int state_char = 0;
int id_msg = 0;

// - Mqtt:
void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, SUB_TOPIC) == 0) {
    String answer;
    //obtem a string do payload recebido
    for (int i = 0; i < length; i++) {
      char c = (char)payload[i];
      answer += c;
    }
    Serial_ArdEsp.println(answer);
    delay(10);
  }
}

String conteudo = "";
void send_odometry(const char* topic) {
  char caractere = Serial_ArdEsp.read();  //read character
  if (caractere == '<') state_char++;
  if (state_char > 1) state_char = 0;
  if (state_char == 1 && caractere != '<' && caractere != '>')
    if (isDigit(caractere) || (caractere == '.') || (caractere == ';') || (caractere == '-'))
      conteudo.concat(caractere);
    else {
      state++;
      state_char = 0;
      conteudo = "";
    }
  if (caractere == '>') {
    state_char = 0;
    state++;
    conteudo = "<" + conteudo + ">";
    //    Serial.println(conteudo);
    client.publish(topic, (char*) conteudo.c_str(), true);
    delay(10);
    conteudo = "";
  }
}

void setup() {
  // Serial communication with PC
  Serial.begin(115200);
  // Serial communication with ESP
  Serial_ArdEsp.begin(115200);
  
  // Liga o LED do Node para saber se ele está vivo
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  
  // Inicializações:
  initWiFi();
  initMQTT();
}

double time_now = millis();

void loop() {
  verifyConections();

  if (state == 0) {
    if (Serial_ArdEsp.available()) {
      send_odometry(PUB_TOPIC);
      time_now = millis();
    }
    else {
      state++;
      time_now = millis();
    }
  }
  else {
    client.loop();
    state = 0;
  }
  if (state >= 2) state = 0;
}
