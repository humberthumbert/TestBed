const express = require('express');
const app = express();
const port = process.env.PORT || 5000;
const { exec } = require('child_process');

const wishList = {
  camera1: 'pixel1.launch',
  camera2: 'pixel2.launch',
  camera3: 'pixel3.launch',
  camera4: 'pixel4.launch',
  face1: 'face1.launch',
  face2: 'face2.launch',
  face3: 'face3.launch',
  face4: 'face4.launch',
  velodyne: 'VLP16_points.launch',
  chained_projection1: 'chained_projection1.launch',
  chained_projection2: 'chained_projection2.launch',
  chained_projection3: 'chained_projection3.launch',
  chained_projection4: 'chained_projection4.launch',
  projection1: 'projection1.launch',
  projection2: 'projection2.launch',
  projection3: 'projection3.launch',
  projection4: 'projection4.launch',
  server: 'video_server.launch',
  offline: `pixor offline_pipeline.launch type:='-kitti' path:='{path}' pc_front:='1' camera0:='0' camera1:='0' camera2:='1' camera3:='0'`,
};

exec('roscore', (error) => {
  if (error) {
    console.error('roscore went a bit crazy', error);
  }
});

const starts = {};
const processes = {};

Object.keys(wishList).forEach(w => { processes[w] = null; starts[w] = false; });

app.use(function (req, res, next) {
  res.header("Access-Control-Allow-Origin", "*"); // update to match the domain you will make the request from
  res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
  next();
});

app.use(express.json()) // for parsing application/json
app.use(express.urlencoded({ extended: true })) // for parsing application/x-www-form-urlencoded

// console.log that your server is up and running
app.listen(port, () => console.log(`Listening on port ${port}`));

// POST method route
app.get('/open', function (req, res, next) {
  const paramsWishedFor = req.query.launchers.split(',');
  let path;
  if (paramsWishedFor.includes('offline')) {
    path = req.query.offlinepath;
    wishList.offline = wishList.offline.replace('{path}', path);
  }
  paramsWishedFor.forEach((param, i) => {
    let cmd = param === 'offline' ? `roslaunch ${wishList.offline}` : `roslaunch ../launchers/${wishList[param]}`;
    if (!starts[param]) {
      if (i > 0) {
        sleep(1000).then(() => {
          processes[param] = exec(cmd, (error) => {
            if (error) {
              console.error(`exec error: ${error}`);
              res.send('error');
            }
          });
        });
      } else {
        processes[param] = exec(cmd, (error) => {
          if (error) {
            console.error(`exec error: ${error}`);
            res.send('error');
          }
        });
      }
    }
  });  

  // Usage!
  sleep(10000).then(() => {
    paramsWishedFor.forEach(param => { starts[param] = true; });
    res.send("hello world");
  });

});

// sleep time expects milliseconds
function sleep (time) {
  return new Promise((resolve) => setTimeout(resolve, time));
};

app.get('/stop', function (req, res, next) {
  Object.keys(starts)
  .filter(p => starts[p] && processes[p])
  .forEach(p => { starts[p] = false; });

  exec('killall roslaunch', (error) => { if (error) { console.error(error); } });
  res.send('Stopped');
});