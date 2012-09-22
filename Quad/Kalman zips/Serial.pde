
 #include <SoftwareSerial.h>
 #include <Time.h>
#define teamA_rxpin 6
#define teamA_txpin 7
#define teamB_rxpin 4
#define teamB_txpin 5

SoftwareSerial serial_teamA = SoftwareSerial(teamA_rxpin, teamA_txpin);
SoftwareSerial serial_teamB = SoftwareSerial(teamB_rxpin, teamB_txpin);
int teamAtimes[32];
int teamBtimes[32];
int team = -1;
int turn = 0;
boolean solo_play = 0;
  

void setup() {
  // initialize both serial ports:
    UCSR0C =(1<<USBS0) | (3<<UCSZ00);
  Serial.begin(9600);
    pinMode(teamA_rxpin, INPUT);
  pinMode(teamA_txpin, OUTPUT);
    pinMode(teamB_rxpin, INPUT);
  pinMode(teamB_txpin, OUTPUT);
  serial_teamA.begin(9600);
  serial_teamB.begin(9600);
  //Serial1.begin(9600);
}

void loop() {
  // read from port 0, send to port 0:

     char game = -1;
     while(game != 'R' && game != 'r' && game != 'N' && game != 'n')
     {
       Serial.println("(N)ew game, or (R)esume game?");
       while(Serial.available() == 0); // loop until data
       game = Serial.read();
       Serial.print(game);
       Serial.println(" received"); 
      }
      
     char mode = -1;
     while(mode != 'M' && mode != 'm' && mode != 'H' && mode != 'h')
     {
       Serial.println("(M)achine, or (H)uman opponent?");
       while(Serial.available() == 0); // loop until data
       mode = Serial.read();
       Serial.print(mode);
       Serial.println(" received");
       
      }
     if(mode == 'h' || mode == 'H')
       solo_play = true;
       else
       solo_play = false;
     
     
     char temp = -1;
     while(temp != '1' && temp != '0' && temp != 'r')
     {
       Serial.println("Enter starting team-or random: (0,1,r)");
       while(Serial.available() == 0); // loop until data
       temp = Serial.read();
       Serial.print(temp);
       Serial.println(" received");
       
      }
      if(temp == '0')
        team = 0;
      else if(temp == '1')
        team = 1;
     else if(temp == 'r')
      {
        randomSeed(analogRead(0));
        double r = random(0,2);  
        if(r<.5)
        {
           team = 0;
        }
        else
        {
          team = 1;
        }
      }
      if(solo_play)
          team = team *2; //0 and 2
      Serial.print("First player is ");
      Serial.println(team);
      
      if(game == 'n' || game == 'N')
        start_new_game(team);
      if(game == 'r' || game == 'R')//do a resume game. (Not implemented)
        team = start_resume_game(team);
     for(turn = 0; turn < 44; turn++) // go higher in popout games?
     {
       //time_t before = now();
       long before = millis();
       char c = await_play(team);
       long after = millis();
       //time_t after = now();
       int tim = (int)(after-before);
       Serial.print("Time for the last move: ");
       Serial.println(tim);
       if(team == 0)
         teamAtimes[(int)floor(turn%2)] = tim;
       else
         teamBtimes[(int)floor(turn%2)] = tim; 
       
       Serial.println("Send any char for next turn");
       while(Serial.available() == 0); // loop until data
       Serial.read();
       
        if(solo_play)
          team = 2-team;
        else
          team = 1 -team;
          send_play(team,c);
        
     }
      
     int col = 0;
     for(int i = 0; i < 10; i++)
    {
 
     col = (int)(random('1', '8'));
     Serial.print(col,BYTE);
     
       while(Serial.available() == 0); //spin-lock till availabale
       Serial.read();
     } 
     
    
  
}

int start_resume_game(int team)
{
    serial_teamA.print('R', BYTE);
    serial_teamB.print('R', BYTE);
    Serial.print('R', BYTE);
    delay(500);
    
    if( team== 0)
    {
      serial_teamA.print('s', BYTE);
      serial_teamB.print('w', BYTE);
    }
    else
    {
      serial_teamA.print('w',BYTE);
      serial_teamB.print('s',BYTE);
    }
    char c = '0';
    Serial.print("Enter chip moves, starting with team: ");
    Serial.println(team);
    Serial.print("Enter 'd' when done");
    int count = 0;
    while(c != 'd') // wait until done is sent
    {
      while(Serial.available() == 0);//read chars from TA machine
      c = Serial.read();
      if (c <='7' && c>='1' && c != 'd')
      {
        send_play(0,c);//team A
        send_play(1,c);//team B
        send_play(2,c);//TA
        count++; // keep track of how many chips played
      }
      else
      {
        Serial.print(c);
        Serial.println(" received, invalid, try again.");
      }
    }
    if(count % 2 == 1) //odd number of chips played
    {
      if(solo_play)
        team = 2-team; //opposite team starts
      else
        team = 1-team;
    }
    else
    {;}//same team starts
     
    //all previously played chips entered, begin play
    if( team== 0)
    {
      serial_teamA.print('S', BYTE);
      serial_teamB.print('W', BYTE);
    }
    else
    {
      serial_teamA.print('W',BYTE);
      serial_teamB.print('S',BYTE);
    }
    
    Serial.println("Resume Game commands sent");
    return team;
}
void start_new_game(int team)
{
    serial_teamA.print('N', BYTE);
    serial_teamB.print('N', BYTE);
    Serial.print('N', BYTE);
    //wait 1 minute between N and S
   // delay(60000);//serial library gets confused if sent too quickly
    delay(6000);
    
    if( team== 0)
    {
      serial_teamA.print('S', BYTE);
      serial_teamB.print('W', BYTE);
    }
    else
    {
      serial_teamA.print('W',BYTE);
      serial_teamB.print('S',BYTE);
    }
    Serial.println("New Game commands sent");
}

char await_play(int team)
{
  char ret;
  Serial.print("Awaiting move from team ");
  Serial.println(team);
    if( team == 0)
    {
      ret = serial_teamA.read();
      if(ret <'1' || ret > '7') //invalid reg move
      {}
      if(ret < 'a' || ret > 'g') //invalid popout move
      {}
    }
    else if(team == 1)
    {
      ret = serial_teamB.read();
      if(ret <'1' || ret > '7') //invalid reg move
      {}
      if(ret < 'a' || ret > 'g') //invalid popout move
      {}
    }
    else if (team == 2)
    {
      while(Serial.available() == 0); // loop until data
       ret = Serial.read();
      if(ret <'1' || ret > '7') //invalid reg move
      {}
      if(ret < 'a' || ret > 'g') //invalid popout move
      {}
      
    }
    return ret;
}

char send_play(int team,char signal)
{
    if( team == 0)
    {
     serial_teamA.print(signal, BYTE);
    }
    else if (team == 1)
    {
     serial_teamB.print(signal,BYTE);
    }
    //else if (team ==2)
    //always report move to computer
    {
     Serial.print(team);
     Serial.print("'s opponent played in column ");
     Serial.println(signal,BYTE); 
    }
}
