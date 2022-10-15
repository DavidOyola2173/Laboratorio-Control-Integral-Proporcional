/*Codigo Motor-Generador Dc con Controladores:
 * Autores: David Santiago Oyola Lozano
 * Diego Fernando Blanco Rueda
 * Heiner Camilo Varela
*/

//DEFINICION DE PINES DE LOS COMPONENTES:-----------------------------------------------------------------------------------------------------

//Encoder:
#define encoder 2

//Puente H:
int ENA = 11;
int IN1 = 8;
int IN2 = 9;

//DEFINICION DE VARIABLES:--------------------------------------------------------------------------------------------------------------------
//Lectura Analogica del Encoder:
volatile unsigned muestreoactualinterrupcion=0; //El tiempo actual del muestreo de la interrupcion.
volatile unsigned muestreoanteriorinterrupcion=0; //El tiempo anterior al tiempo actual del muestreo de interrupcion.
volatile unsigned deltamuestreointerrupcion=0;  //El diferencial de tiempo entre el tiempo anterior y actual de muestreo de interrupcion.

//Variables del calculo de velocidad angular:
int ciclo=0;  //Contador
double frecuencia=0;  //La frecuencia de interrupcion calculada por el promedio de deltas de tiempo.
double w=0; //Velocidad angular calculada del encoder.

//Variables para tiempo de muestro:
float  tiempo1=0;
float  tiempo2=0;
float  tiempo=0;
double n=1; //Numero de interrupciones (o ranuras) del medidor del encoder.

//Vector de suma promnedio:
float vector[]={0,0,0,0,0,0,0,0,0,0}; //Vector donde se guarda cada delta de tiempo para hacer un promedio.
int tam=10; //Tamaño de evaluacion de los ciclos.

//Parametros de control de PWM:                        
int  PWM=0;  //PWM que se controla.
float lecturaAnalogica =0; //Lectura analogica motor.
float voltajeSalida = 0; //Voltaje Generador.

//Parametros controlador:
float sp; //Valor del setpoint
float pv; //Valor de la variable del proceso
float en;
float un = 0;
float Uc;
float K;           //Ganancia
float Umax = 100;         //Acción de control máxima
float Umin = 0;         //Acción de control mínimo
float un_1 =0;      //Elemento de memoria del integrador


//INICIO DEL CODIGO:-------------------------------------------------------------------------------------------------------------------------
//Funcion del encoder:
void encoderVoid(){ //Esta parte del codigo entra a funcionar cuando sucede una interrupcion en el encoder
ciclo++;  //Inicia el tick.

//En esta parte se mide el diferencial de tiempo entre tick y tick medido por el encoder para sacar un promedio de tiempo:
if(ciclo==1){
      float media=0;  //Se inicializa la variable para obtener el promedio de los delta de tiempos
      deltamuestreointerrupcion=muestreoactualinterrupcion-muestreoanteriorinterrupcion;//Se hace el diferencial de tiempo de interrupcion de ticks del encoder.
      for(int i=0;i<tam-1;i++){ //Ciclo for para crear un vector de n posiciones.
        vector[i]=vector[i+1];  //Este vector se utiliza para almacenar los delta de tiempos.
      }
      vector[tam-1]=deltamuestreointerrupcion;
      for(int i=0;i<tam;i++){ //Ciclo for para comenzar a hacer el promedio de tiempos de interrupcion.
        media=vector[i]+media;
      }
      media=media/tam;  //Se divide entre la cantidad de datos para asi obtener el promedio.
      deltamuestreointerrupcion=media;
      muestreoanteriorinterrupcion=muestreoactualinterrupcion;//Se actualiza el tiempo de interrupcion anterior
    }
ciclo=0;  //Deja el tick en 0
muestreoactualinterrupcion=millis();  //Actualiza el muestreo actual.
    
    //if(voltajeSalida==0.00){n=0;}
    
//Calculo de la frecuencia angular:
    if(deltamuestreointerrupcion!=0){ //Según el promedio de tiempo sacado anteriormente se pasa a frecuencia en radianes por segundo:
      frecuencia=(1000*n)/(double)deltamuestreointerrupcion;
    }else{
      frecuencia=0;
    }
    //Se calcula la velocidad angular siendo 2*pi*frecuencia:
    w=(2*3.141516)*frecuencia;
}

//Funcion de los controladores (ON-OFF, Proporcional, Integral y el por defecto):
void controladores(int tipoControl, int variableProcesoSeleccionada){
  //Se selecciona la variable del proceso a controlar:
  switch(variableProcesoSeleccionada){
    case 1:
      pv = voltajeSalida;
      break;
    case 2:
      pv = w;
      break;
    default:
      pv = voltajeSalida;
      break;
  }
    
  //Controladores:
  switch(tipoControl){
    case 1: //Control ON-OFF:
      if(Serial.available()>0){
       String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
       sp=entradaSerial.toFloat();
      }
      en = sp-pv;

      if(en > 0){
        un = Umax;
      }else{
        un = Umin;
      }
      PWM = un;
    break;
    case 2: //Control Proporcional:
      if(Serial.available()>0){
       String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
       sp=entradaSerial.toFloat();
      }
      K = 28;
      en = sp-pv;
      Uc = K*en;

      un = Uc;

      if(Uc < Umin){
        un = Umin;
      }
      if(Uc>Umax){
        un = Umax;
      }
      PWM = un;
    break;
    case 3: //Control Integral:
      if(Serial.available()>0){
       String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el setpoint a tomar.
       sp=entradaSerial.toFloat();
      }
      K = 0.35;
      en = sp-pv;
      Uc = K*en+un_1;
    
      un = Uc;
    
      if(Uc<Umin){
        un = Umin;
      }
      if(Uc>Umax){
        un = Umax;
      }
      PWM = un;
      un_1 = un;
    break;
    default:  //Control por defecto (setpoint de PWM Manualmente):
      if(Serial.available()>0){
       String entradaSerial = Serial.readStringUntil('\n'); //Obtiene el valor ingresado para que sea el PWM a tomar.
       PWM=entradaSerial.toFloat();
      }
    break;
  }
}

void setup() {
  //Activacion de la interrupcion en flanco de bajada del encoder:
  attachInterrupt(digitalPinToInterrupt(encoder),encoderVoid,FALLING);
  Serial.begin(9600); //Monitor Serial en 9600 baudios.
  tiempo1=millis();
  
  //Se definen las entradas y salidas de los pines:
  pinMode(encoder, INPUT);  //Se necesita el pin del encoder como entrada para la interrupcion.
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); //Se define los pines del puente H como Salidas.
  pinMode(IN2, OUTPUT);
  
  //Señal Puente H L298N:
  digitalWrite(IN1,HIGH); //Se ajustan IN1 e IN2 para que gire en sentido horario el motor.
  digitalWrite(IN2,LOW);  //Si se quiere girar antihorario definir IN1=LOW e IN2=HIGH.
}

void loop() {
  lecturaAnalogica=analogRead(A5); //Se lee el voltaje del pin analogico A5 para mensurar el voltaje de salida del generador.
  voltajeSalida=lecturaAnalogica/1023*5.0;
  if(PWM < 35.0){
    w = 0;
  }
  //Llama a la función controladores:
  //Instrucciones: controladores(ControladorAUsar, VariableDelProcesoASeleccionar);
  //Como ControladorAUsar se pueden seleccionar las siguientes opciones = 1: Control ON-OFF; 2: Control Proporcional, 3: Control Integral, OtroNumero: Por Defecto.
  //Como VariableDelProcesoASeleccionar se pueden seleccionar las siguientes opciones = 1: Selecciona el voltaje como variable del proceso, 2: Selecciona la velocidad como variable del proceso. 
  controladores(0,1);
  analogWrite(ENA,map(PWM,0,100,0,255)); //Enviamos el PWM al puente H.
  
  tiempo2=millis();
  
  if(tiempo2>=tiempo1+15){  //Se obtendrá las muestras de velocidad, voltaje y PWM para los instantes de tiempo correspondientes al tiempo de muestreo.
    tiempo1 = millis();   //En este caso se ingresa el tiempo de muestreo en el if.
    tiempo = tiempo1/1000;
    //Salida de monitor Serial para imprimir los datos: 
    Serial.print(" "); Serial.print(w); Serial.print(" ");  //Velocidad en RPM.
    Serial.print(" "); Serial.print(voltajeSalida); Serial.print(" ");  //Porcentaje de PWM aplicado.
    //Serial.print(" "); Serial.print(sp); Serial.print(" ");  //Porcentaje de PWM aplicado.
    Serial.print(" "); Serial.print(PWM*5/100); Serial.print(" ");  //Porcentaje de PWM aplicado.
    Serial.println();
  }
}
