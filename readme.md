## How to setup for running scripts

- First create 3.8 python venv => python3.8 -m venv venv38 

- Then activate venv => source venv38/bin/activate

- Then install requirements => pip install -r requirements.txt

- Then run any scripts as required => python <script_name>.py

## To do list:

- Add some catch error handling for when there is no marker !High
    - This needs changes to MoCap Stream error handling to correct

- Get some data to plot error: !High
    - delta angular/rate
    - delta angle
    - ie get angles of target and tracker and write to some .mats or .np
    - then plot the angles over time
    - also calculate the angular rate and plot the angular rate over time
    - looking for minimal deviation in the difference of angular rate between the two
    - also looking for a good tolerance of the difference in angle between the two
    - can investigate cases for which this is worse or better to evaluate what needs improving

- Tune PID to improve control response !High

- Get sample data for flying in circle round target !Med

- Fit tracking control to simulation of this case !Med

- Add camera interfacing and triggering to the code !High