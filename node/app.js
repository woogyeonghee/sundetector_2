const express = require("express");
const mqtt = require('mqtt');
const options = {
    username:"woo",
    password:"12345678",
  };
const client  = mqtt.connect('mqtt://3.38.154.143',options);
const SUNDETECT=require("./models/sundetector.js");
const http = require("http");
const app = express();

const mongoose =require("mongoose");
require ("dotenv").config({path:"variables.env"});
app.set("port","3000");
var server = http.createServer(app);
client.on("connect",()=>{
    console.log("mqtt connect");
    client.subscribe("test");
  });

client.on("message",async( test ,message ) => {
    var obj = JSON.parse(message);
    var date = new Date();
    var year = date.getFullYear();
    var month = date.getMonth();
    var today = date.getDate();
    var hours = date.getHours();
    var minutes = date.getMinutes();
    var seconds = date.getSeconds();
    obj.created_at = new Date(Date.UTC(year,month,today,hours,minutes,seconds));
    console.log(obj);
    const SUN = new SUNDETECT({
      Lf_Rt_pos : obj.Lf_Rt_pos,
      Tp_Dn_pos : obj.Tp_Dn_pos,
      AVG : obj.AVG,
      created_at : obj.created_at
    });
    try{
      const saveSUN = await SUN.save();
      console.log("insert OK");
    }catch(err){
      console.log({message : err});
    }
  });

//web server

server.listen(3000, (err)=>{ 
  if(err){
    return console.log(err);
  }else{
    console.log("server ready");
    //connection to db
    mongoose.connect(process.env.MONGODB_URL,
      { useNewUrlParser: true,useUnifiedTopology: true},
      ()=> console.log('connected to DB')
      );
  }
});