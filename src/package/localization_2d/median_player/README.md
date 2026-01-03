simple_player_server:

    LOOP MODE:

        (mode = 0, sound_id = 0):   Sequential Loop 

        (mode = 0, sound_id = -1):  Random Loop
 
        (mode = 0, sound_id = id):  Single Loop

    ——————

    SINGLE MODE:

        (mode = 1, sound_id = 0):   Single Play, id =1 

        (mode = 1, sound_id = id):  Single Play, id =id


-----------------------------------------------------------------------

-----------------------------------------------------------------------


median_player_server: 


    LOOP MODE:

        (mode = 'loop', sound_id = idx):  Sequential Loop 

        (mode = 'loop', sound_id = id):        Single Loop

    ——————

    RANDOM MODE:
    
         (mode = 'rand', sound_id = 0):    Random Loop    
          
    ——————
    
    SINGLE MODE:

        (mode = 'single', sound_id = id):  Single Play
        
     ——————       
    
    OTHERS:

         Invalid Warning



  ------
  
  Subscribed Topics:

  (1) median_control (std_msgs/Header) :

      > 'pause'

      > 'set_pause'

      > 'play'

      > 'skip'

      > 'up'

      > 'down'

      > 'volume'

  (2) median_player_server/cancel (actionlib_msgs/GoalID)


  ------

  Published Topics:

   none

  ------

  Rosparameters

  (1) median_player_server/volume_step (Volume Step of up/down control. > Default=30)

  (2) median_player_server/median_path (Sound files path)

  (3) median_player_server/if_stop_sofort (As the name implies. > Default=30)




