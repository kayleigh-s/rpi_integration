# Instructions for running the Apprentice Agents Learner

## Description

This repository is used to interface with the docker images developed for our integration work with RPI. Docker images are `leia/ontosem` and `leia/agents`, and they expose a REST API once opened.

Once opened, the REST API is available on `localhost:5002`

   * `/learn` -> `POST` raw JSON in the format for the learner; you can post one or more sequential inputs (utterances and actions); the response will be the HTN in JSON format (the current complete HTN, with the latest inputs included)
   * `/alpha/gettree` -> `GET`; returns the current HTN (without modifying it):
         * use `&format=json` to get the same response format as `/learn`
         * use `&format=pretty` to get a human readable version
   * `/alpha/reset` -> `DELETE`; this will clear the current tree (in lieu of killing and restarting the docker containers)
   * `/alpha/maketree` -> deprecated, ignore this command for now

## Preliminaries

Pull the required images from the docker repo:

```
docker login --username=[username]
docker pull leia/ontosem:1.0.0
docker pull leia/agents:0.5.0
```

## Usage

Run the compose file to turn on 3 containers and their respective services `docker-compose -f ./docker-compose.yml up`


Learner Input Format:
[
  ["u", "Utterance text here."], ["a", "action-name-here"], ...
]
