Instructions for running the Apprentice Agents Learner:

1) pull the required images from the docker repo
   a) docker login --username=[username]
   b) docker pull leia/ontosem:1.0.0
   c) docker pull leia/agents:0.5.0

2) run the compose file to turn on 3 containers and their respective services
   docker-compose -f [path]/docker-compose.yml up

3) The REST API is now available on localhost:5002
   a) /learn - POST raw JSON in the format for the learner (see below); you can post one or more sequential inputs (utterances and actions); the response will be the HTN in JSON format (the current complete HTN, with the latest inputs included)
   b) /alpha/gettree - GET; returns the current HTN (without modifying it);
      use &format=json to get the same response format as /learn; 
      use &format=pretty to get a human readable version
   c) /alpha/reset - DELETE; this will clear the current tree (in lieu of killing and restarting the docker containers)
   d) /alpha/maketree - deprecated, ignore this command for now

Learner Input Format:
[
  ["u", "Utterance text here."], ["a", "action-name-here"], ...
]