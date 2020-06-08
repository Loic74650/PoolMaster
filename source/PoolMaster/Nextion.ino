/*
  NEXTION TFT related code
*/

int CurrentPage = 0;
char HourBuffer[8];

//Structure holding the measurement values to display on the Nextion display
struct TFTStruct
{
  float pH, Orp, pHSP, OrpSP, WT, WTSP, AT, PSI;
  uint8_t FSta, FSto, pHTkFill, OrpTkFill;
  boolean Mode, NetW, Filt, Heat, R0, R1, R2;
  unsigned long pHPpRT, OrpPpRT;
} TFTStruc =
{ //default values.
  7.30, 730, 7.4, 740, 27.11, 28.5, 25.5, 0.80,
  8, 20, 100, 100,
  1, 0, 0, 0, 0, 0, 0,
  0, 0
};


//function to update TFT display//it updates the TFTStruct variables, the global variables of the TFT + the widgets of the active page
//call this function at least every second to ensure fluid display
void UpdateTFT()
{
  myNex.NextionListen();

  sprintf(HourBuffer, "%02d:%02d:%02d", hour(), minute(), second());
  myNex.writeStr("page0.vaTime.txt", HourBuffer);

  if (storage.PhValue != TFTStruc.pH)
  {
    TFTStruc.pH = storage.PhValue;
    myNex.writeStr("page0.vapH.txt", String(TFTStruc.pH, 2));
    if (CurrentPage == 0)  myNex.writeStr("pH.txt", String(TFTStruc.pH, 2));
  }
  if (storage.OrpValue != TFTStruc.Orp)
  {
    TFTStruc.Orp = storage.OrpValue;
    myNex.writeStr("page0.vaOrp.txt", String(TFTStruc.Orp, 2));
    if (CurrentPage == 0)  myNex.writeStr("Orp.txt", String(TFTStruc.Orp, 2));
  }
  if (storage.Ph_SetPoint != TFTStruc.pHSP)
  {
    TFTStruc.pHSP = storage.Ph_SetPoint;
    String temp = "(" + String(TFTStruc.pHSP, 1) + ")";
    myNex.writeStr("page0.vapHSP.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("pHSP.txt", temp);
  }
  if (storage.Orp_SetPoint != TFTStruc.OrpSP)
  {
    TFTStruc.OrpSP = storage.Orp_SetPoint;
    String temp = "(" + String((int)TFTStruc.OrpSP) + ")";
    myNex.writeStr("page0.vaOrpSP.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("OrpSP.txt", temp);
  }
  if (storage.TempValue != TFTStruc.WT)
  {
    String temp = String(TFTStruc.WT, 2) + (char)176 + "C";
    TFTStruc.WT = storage.TempValue;
    myNex.writeStr("page0.vaWT.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("W.txt", temp);
  }
  if (storage.TempExternal != TFTStruc.AT)
  {
    String temp = String(TFTStruc.AT, 2) + (char)176 + "C";
    TFTStruc.AT = storage.TempExternal;
    myNex.writeStr("page0.vaAT.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("A.txt", temp);
  }
  if (storage.PSIValue != TFTStruc.PSI)
  {
    String temp = String(TFTStruc.PSI, 2) + "b";
    TFTStruc.PSI = storage.PSIValue;
    myNex.writeStr("page0.vaPSI.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("P.txt", temp);
  }
  if ((ChlPump.UpTime != TFTStruc.OrpPpRT) || ((int)(storage.ChlFill - ChlPump.GetTankUsage()) != TFTStruc.OrpTkFill))
  {
    TFTStruc.OrpPpRT = ChlPump.UpTime;
    TFTStruc.OrpTkFill = (int)(storage.ChlFill - ChlPump.GetTankUsage());

    String temp = String(TFTStruc.OrpTkFill) + (char)37 + " / " + String(float(TFTStruc.OrpPpRT / 1000 / 60), 1) + "min";
    myNex.writeStr("page0.vaOrpTk.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("OrpTk.txt", temp);
  }
  if ((PhPump.UpTime != TFTStruc.pHPpRT) || ((int)(storage.AcidFill - PhPump.GetTankUsage()) != TFTStruc.pHTkFill))
  {
    TFTStruc.pHPpRT = PhPump.UpTime;
    TFTStruc.pHTkFill = (int)(storage.AcidFill - PhPump.GetTankUsage());

    String temp = String(TFTStruc.pHTkFill) + (char)37 + " / " + String(float(TFTStruc.pHPpRT / 1000 / 60), 1) + "min";
    myNex.writeStr("page0.vapHTk.txt", temp);
    if (CurrentPage == 0)  myNex.writeStr("pHTk.txt", temp);
  }

  if (storage.AutoMode != TFTStruc.Mode)
  {
    TFTStruc.Mode = storage.AutoMode;
    String temp = TFTStruc.Mode ? "AUTO" : "MANU";
    myNex.writeNum("page1.vabMode.val", storage.AutoMode);
    if (CurrentPage == 0)
    {
      myNex.writeStr("p0Mode.txt", temp);
    }
    else if (CurrentPage == 1)
    {
      myNex.writeStr("p1Mode.txt", temp);
      myNex.writeStr("t1Mode.txt", temp);
      if (storage.AutoMode == 1)
        myNex.writeStr("bMode.pic=8");
      else
        myNex.writeStr("bMode.pic=7");
    }
    else if (CurrentPage == 2)
    {
      myNex.writeStr("p2Mode.txt", temp);
    }
    else if (CurrentPage == 3)
    {
      myNex.writeStr("p3Mode.txt", temp);
    }
  }

  if (FiltrationPump.IsRunning() != TFTStruc.Filt)
  {
    TFTStruc.Filt = FiltrationPump.IsRunning();
    myNex.writeNum("page1.vabFilt.val", TFTStruc.Filt);
    if (CurrentPage == 1)
    {
      if (TFTStruc.Filt == 1)
        myNex.writeStr("bFilt.pic=8");
      else
        myNex.writeStr("bFilt.pic=7");
    }
  }

  if (storage.WaterHeat != TFTStruc.Heat)
  {
    TFTStruc.Heat = storage.WaterHeat;
    myNex.writeNum("page1.vabHeat.val", TFTStruc.Heat);
    if (CurrentPage == 1)
    {
      if (TFTStruc.Heat == 1)
        myNex.writeStr("bHeat.pic=8");
      else
        myNex.writeStr("bHeat.pic=7");
    }
  }

  if (digitalRead(RELAY_R1) != TFTStruc.R0)
  {
    TFTStruc.R0 = digitalRead(RELAY_R1);
    myNex.writeNum("page1.vabR0.val", TFTStruc.R0);
    if (CurrentPage == 1)
    {
      if (TFTStruc.R0 == 1)
        myNex.writeStr("bR0.pic=8");
      else
        myNex.writeStr("bR0.pic=7");
    }
  }

  if (digitalRead(RELAY_R2) != TFTStruc.R1)
  {
    TFTStruc.R1 = digitalRead(RELAY_R2);
    myNex.writeNum("page1.vabR1.val", TFTStruc.R1);
    if (CurrentPage == 1)
    {
      if (TFTStruc.R1 == 1)
        myNex.writeStr("bR1.pic=8");
      else
        myNex.writeStr("bR1.pic=7");
    }
  }

  if (digitalRead(RELAY_R6) != TFTStruc.R2)
  {
    TFTStruc.R2 = digitalRead(RELAY_R6);
    myNex.writeNum("page1.vabR2.val", TFTStruc.R2);
    if (CurrentPage == 1)
    {
      if (TFTStruc.R2 == 1)
        myNex.writeStr("bR2.pic=8");
      else
        myNex.writeStr("bR2.pic=7");
    }
  }
  
  switch (CurrentPage)
  {
    case 0: {
        myNex.writeStr("p0Time.txt", HourBuffer);
        break;
      }
    case 1: {
        myNex.writeStr("p1Time.txt", HourBuffer);
        break;
      }
    case 2: {
        myNex.writeStr("p2Time.txt", HourBuffer);
        break;
      }
    case 3: {
        myNex.writeStr("p3Time.txt", HourBuffer);
        break;
      }
  }
}

void UpdateToggleButton(String ButtonName)
{
  if (ButtonName == F("bMode"))
  {
    if (TFTStruc.Mode)
    {
      myNex.writeNum(F("bMode.pic"), 8);
      myNex.writeNum(F("vabMode.val"), 1);
      myNex.writeStr("t1Mode.txt", F("AUTO"));
    } else
    {
      myNex.writeNum(F("bMode.pic"), 7);
      myNex.writeNum(F("vabMode.val"), 0);
      myNex.writeStr("t1Mode.txt", F("MANU"));
    }
  }
  else if (ButtonName == F("bFilt"))
  {
    if (TFTStruc.Filt)
    {
      myNex.writeNum(F("bFilt.pic"), 8);
      myNex.writeNum(F("vabFilt.val"), 1);
    } else
    {
      myNex.writeNum(F("bFilt.pic"), 7);
      myNex.writeNum(F("vabFilt.val"), 0);
    }
  }
  else if (ButtonName == F("bHeat"))
  {
    if (TFTStruc.Heat)
    {
      myNex.writeNum(F("bHeat.pic"), 8);
      myNex.writeNum(F("vabHeat.val"), 1);
    } else
    {
      myNex.writeNum(F("bHeat.pic"), 7);
      myNex.writeNum(F("vabHeat.val"), 0);
    }
  }
  else if (ButtonName == F("bR0"))
  {
    if (TFTStruc.R0)
    {
      myNex.writeNum(F("bR0.pic"), 8);
      myNex.writeNum(F("vabR0.val"), 1);
    } else
    {
      myNex.writeNum(F("bR0.pic"), 7);
      myNex.writeNum(F("vabR0.val"), 0);
    }
  }
  else if (ButtonName == F("bR1"))
  {
    if (TFTStruc.R1)
    {
      myNex.writeNum(F("bR1.pic"), 8);
      myNex.writeNum(F("vabR1.val"), 1);
    } else
    {
      myNex.writeNum(F("bR1.pic"), 7);
      myNex.writeNum(F("vabR1.val"), 0);
    }
  }
  else if (ButtonName == F("bR2"))
  {
    if (TFTStruc.R2)
    {
      myNex.writeNum(F("bR2.pic"), 8);
      myNex.writeNum(F("vabR2.val"), 1);
    } else
    {
      myNex.writeNum(F("bR2.pic"), 7);
      myNex.writeNum(F("vabR2.val"), 0);
    }
  }
}

void ResetTFT()
{
  myNex.writeStr("rest");
}

//Page 0 has finished loading
void trigger1()
{
  CurrentPage = 0;
  //UpdateTFTDisplay(0);
  DEBUG_PRINT("Nextion p0");
}

//Page 1 has finished loading
void trigger2()
{
  CurrentPage = 1;
  //UpdateTFTDisplay(0);
  DEBUG_PRINT("Nextion p1");
}

//Page 2 has finished loading
void trigger3()
{
  CurrentPage = 2;
  //UpdateTFTDisplay(0);
  DEBUG_PRINT("Nextion p2");
}

//Page 3 has finished loading
void trigger4()
{
  CurrentPage = 3;
  //UpdateTFTDisplay(0);
  DEBUG_PRINT("Nextion p3");
}

//MODE button was toggled
void trigger5()
{
  TFTStruc.Mode = (boolean)myNex.readNumber(F("vabMode.val"));
  DEBUG_PRINT("MODE button");
  if (TFTStruc.Mode)
  {
    String Cmd = F("{\"Mode\":1}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Mode\":0}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//FILT button was toggled
void trigger6()
{
  TFTStruc.Filt = (boolean)myNex.readNumber(F("vabFilt.val"));
  DEBUG_PRINT("FILT button");
  if (TFTStruc.Filt)
  {
    String Cmd = F("{\"FiltPump\":1}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"FiltPump\":0}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//HEAT button was toggled
void trigger7()
{
  TFTStruc.Heat = (boolean)myNex.readNumber(F("vabHeat.val"));
  DEBUG_PRINT("HEAT button");
  if (TFTStruc.Heat)
  {
    String Cmd = F("{\"Heat\":1}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Heat\":0}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 0 button was toggled
void trigger8()
{
  TFTStruc.R0 = (boolean)myNex.readNumber(F("vabR0.val"));
  DEBUG_PRINT("Relay 0 button");
  if (TFTStruc.R0)
  {
    String Cmd = F("{\"Relay\":[1,1]}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[1,0]}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 1 button was toggled
void trigger9()
{
  TFTStruc.R1 = (boolean)myNex.readNumber(F("vabR1.val"));
  DEBUG_PRINT("Relay 1 button");
  if (TFTStruc.R1)
  {
    String Cmd = F("{\"Relay\":[2,1]}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[2,0]}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
}

//Relay 2 button was toggled
void trigger10()
{
  TFTStruc.R2 = (boolean)myNex.readNumber(F("vabR2.val"));
  DEBUG_PRINT("Relay 2 button");
  if (TFTStruc.R2)
  {
    String Cmd = F("{\"Relay\":[6,1]}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
  else
  {
    String Cmd = F("{\"Relay\":[6,0]}");
    queue.push(Cmd);
    DEBUG_PRINT(Cmd);
  }
}
