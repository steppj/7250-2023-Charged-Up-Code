// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class MusicCommand extends CommandBase {
      /* The orchestra object that holds all the instruments */
      Orchestra _orchestra;
      //Grabs motors from swerves
    private Swerve s_Swerve; 
    /* An array of songs that are available to be played*/
    String[] _songs = new String[] {
      "mariov2.chrp",
      "rick.chrp",
      "pullup.chrp",
      "god is good.chrp",
      "geodash.chrp",
      "moogcityminecraft.chrp",
      "nyancat.chrp",
      "countryroads.chrp",
      "freebird.chrp",
    };
  
      /* track which song is selected for play */
      int _songSelection = 0;
  
      /* overlapped actions */
      int _timeToPlayLoops = 0;
  
      /* joystick vars */
      Joystick _joy;
      int _lastButton = 0;
      int _lastPOV = 0;
  
  public TalonFX[] MusicSwerves = new TalonFX[4];  
  /** Creates a new MusicCommand. */
  public MusicCommand(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
  }

      //------------- joystick routines --------------- //
    /** @return 0 if no button pressed, index of button otherwise. */
    int getButton() {
      for (int i = 1; i < 9; ++i) {
          if (_joy.getRawButton(i)) {
              return i;
          }
      }
      return 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    for(int i = 0; i < 4; i++)
    {
      MusicSwerves[i] = s_Swerve.mSwerveMods[i].mAngleMotor;
    }
    /* A list of TalonFX's that are to be used as instruments */
    ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
      
    /* Initialize the TalonFX's to be used */
    for (int i = 0; i < MusicSwerves.length; ++i) 
    {
      _instruments.add(MusicSwerves[i]);
    }
    /* Create the orchestra with the TalonFX instruments */
    _orchestra = new Orchestra(_instruments);
    _joy = new Joystick(0);
            
        /* load whatever file is selected */
        LoadMusicSelection(0);
    System.out.println("Jukebox initalized!");
  }
  void LoadMusicSelection(int offset)
  {
      /* increment song selection */
      _songSelection += offset;
      /* wrap song index in case it exceeds boundary */
      if (_songSelection >= _songs.length) {
          _songSelection = 0;
      }
      if (_songSelection < 0) {
          _songSelection = _songs.length - 1;
      }
      /* load the chirp file */
      _orchestra.loadMusic(_songs[_songSelection]); 

      /* print to console */
      System.out.println("Song selected is: " + _songs[_songSelection] + ".  Press left/right on d-pad to change.");
      
      /* schedule a play request, after a delay.  
          This gives the Orchestra service time to parse chirp file.
          If play() is called immedietely after, you may get an invalid action error code. */
      _timeToPlayLoops = 10;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* poll gamepad */
    int btn = getButton();
    int currentPOV = _joy.getPOV();

    /* if song selection changed, auto-play it */
    if (_timeToPlayLoops > 0) {
        --_timeToPlayLoops;
        if (_timeToPlayLoops == 0) {
            /* scheduled play request */
            System.out.println("Auto-playing song.");
            _orchestra.play();
        }
    }


    /* has a button been pressed? */
    if (_lastButton != btn) 
    {
        _lastButton = btn;

      switch (btn) {
        case 1: /* toggle play and paused */
          if (_orchestra.isPlaying()) {
            _orchestra.pause();
              System.out.println("Song paused");
          } else {
            _orchestra.play();
            System.out.println("Playing song...");
          }
          break;      
            case 2: /* toggle play and stop */
                if (_orchestra.isPlaying()) {
                    _orchestra.stop();
                    System.out.println("Song stopped.");
                }  else {
                    _orchestra.play();
                    System.out.println("Playing song...");
                }
                break;
              }
            }
    /* has POV/D-pad changed? */
    if (_lastPOV != currentPOV) {
      _lastPOV = currentPOV;

      switch (currentPOV) {
          case 90:
              /* increment song selection */
              LoadMusicSelection(+1);
              break;
          case 270:
              /* decrement song selection */
              LoadMusicSelection(-1);
              break;
      }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
