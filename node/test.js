
const mongoose = require("mongoose");
const MONGODB_URL = "mongodb+srv://woo:yuqingxi0516-@cluster0.lllh9.mongodb.net/sundetector?retryWrites=true&w=majority";

mongoose.connect(MONGODB_URL,
    { useNewUrlParser: true,useUnifiedTopology: true},
    (err)=> {
    if(err){
        console.log(err);
    }else{
        console.log('connected to DB');
    }        
    }
);