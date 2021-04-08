TEST RATE:

rosrun rosneuro_ratecheck test_rate _sampling_freq:=590 _sub_topic_data:=/neurodata

TEST DELAY: 

rosrun rosneuro_ratecheck test_delay _expected_delay:=0.0 _first_sub_topic:=/neurodata _second_sub_topic:=/neuroprediction
