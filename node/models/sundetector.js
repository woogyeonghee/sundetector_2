//make schema
//{"Lf_Rt_pos":%4d,"Tp_Dn_pos":%4d,"AVG":""}
//"{\"Lf_Rt_pos\":%4d,\"Tp_Dn_pos\":%4d,\"AVG\":%4d}"
const mongoose =require("mongoose");
const {Schema}=mongoose;
const PosnCds = new Schema(
    {
    Lf_Rt_pos : {
        type : String,
        retuired : true
    },

    Tp_Dn_pos : {
        type : String,
        retuired : true 
    },

    AVG : {
        type : String,
        retuired : true 
    },

    created_at : {
        type :Date,
        default :Date.now
    }

});
  module.exports = mongoose.model('sundetector',PosnCds);
