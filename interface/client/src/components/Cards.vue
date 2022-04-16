<template>
  <b-container>
    <b-row>
      <b-button
        :disabled="connected"
        v-on:click="connect()"
        variant="primary"
        >Connect</b-button
      >

      <b-button
        :disabled="!connected"
        v-on:click="disconnect()"
        variant="primary"
        >Disconnect</b-button
      >
    </b-row>
    <p />

    <b-row>
      <b-col l="4">
        <b-card
          title="Tracks"
          img-src="../assets/img/tracks.png"
          img-alt="Image"
          img-top
          tag="article"
          style="max-width: 20rem"
          class="mb-2"
        >
          <b-card-text>Track actions</b-card-text>
          <p />
          <b-button
            :disabled="!connected"
            :pressed.sync="headlights"
            variant="primary"
            >Turn headlights {{ headlightActionStr() }}</b-button
          >
          <p />
            Track joystick
            <div id="joystick"></div>
          <p />
          
          <b-table stacked :items="[track_state]"></b-table>
        </b-card>
      </b-col>

      <b-col l="4">
        <b-card
          title="Middle"
          img-src="../assets/img/middle.png"
          img-alt="Image"
          img-top
          tag="article"
          style="max-width: 20rem"
          class="mb-2"
        >
          <b-card-text>Middle actions</b-card-text>
          <p />
          <!-- Current state: {{ middle_state }} -->
          <b-table stacked :items="[middle_state]"></b-table>
        </b-card>
      </b-col>

      <b-col l="4">
        <b-card
            title="Camera"
            img-src="../assets/img/selfie.png"
            img-alt="Image"
            img-top
            tag="article"
            style="max-width: 20rem"
            class="mb-2"
          >
          <p />
          Video feed:
          <p />
          <b-img id="vid-feed" src rounded fluid alt=""></b-img>
          <p />
          Depth feed:
          <p />
          <b-img id="depth-feed" src rounded fluid alt=""></b-img>
        </b-card>
      </b-col>
    </b-row>
    <b-row>
      <b-col l="4">
        <b-card
            title="Voice"
            img-src="../assets/img/voice.png"
            img-alt="Image"
            img-top
            tag="article"
            style="max-width: 20rem"
            class="mb-2"
          >
          <b-card-text>Voice actions</b-card-text>
          <p />
          <b-button
            :disabled="!voice_ready"
            v-on:click="speak()"
            variant="primary"
            >Speak to me</b-button
          >
          <b-form-input v-model="tts_input" placeholder="Hello. I am Clare and I do not like aubergines"></b-form-input>
          <p />
          What I have heard so far:
          <p />
          <b-form-textarea
            id="textarea"
            v-model="stt_output"
            placeholder="No data yet..."
            rows="3"
            max-rows="6"
          ></b-form-textarea>
        </b-card>
      </b-col>

      <b-col l="4">
        <b-card
            title="Top"
            img-src="../assets/img/top.png"
            img-alt="Image"
            img-top
            tag="article"
            style="max-width: 20rem"
            class="mb-2"
          >
          <b-card-text>Top actions</b-card-text>
          <p />
          <b-button
            v-on:click="triggerFan()"
            variant="primary"
            >Trigger Fan</b-button
          >
          <p />
            <div>
              <label for="neck-z-angle">Neck z/up/down: {{ neck_z_angle }}</label>
              <b-form-input id="neck-z-angle" v-model="neck_z_angle" type="range" :value="neck_z_angle" min="0" max="100"></b-form-input>
            </div>
            <div>
              <label for="neck-y-angle">Neck y/left/right: {{ neck_y_angle }}</label>
              <b-form-input id="neck-y-angle" v-model="neck_y_angle" type="range" :value="neck_y_angle" min="0" max="100"></b-form-input>
            </div>
            <div>
              <b-form-select v-model="lightring_val" :options="lightring_vals"></b-form-select>
            </div>
          <p />
          <b-table stacked :items="[top_state]"></b-table>
          <p />
        </b-card>
      </b-col>

      <b-col l="4">
        <b-card
            title="Arms"
            img-src="../assets/img/arms.png"
            img-alt="Image"
            img-top
            tag="article"
            style="max-width: 20rem"
            class="mb-2"
          >
          <b-card-text>Arms actions</b-card-text>
          <p />
          <div v-for="(val, name) in arm_servos" :key="name">
            <label :for="name">{{ name }}: {{ val }}</label>
            <b-form-input :id="name" v-model="arm_servos[name]" type="range" :value="val" min="-1" max="100"></b-form-input>
          </div>
        </b-card>
      </b-col>

    </b-row>
  </b-container>
</template>

<script>
import axios from "axios";
import nipplejs from "nipplejs";
import _ from "lodash";

var api = process.env.VUE_APP_API_LOCATION;

// EventSources
var tracksStateStream = null;
var middleStateStream = null;
var topStateStream = null;
var sttStream = null;
//var headStateStream = null;

var joystick = null;

function setupEventSource(src, name, handler) {
  src.addEventListener('open', (e) => {
    console.log("EventSource " + name + " opened");
    console.log(e);
  }, false);

  src.addEventListener('error', (e) => {
    console.log("Error in EventSource " + name);
    console.log(e);
    if (e.readyState == EventSource.CLOSED) {
      console.log("EventSource " + name + " closed");
    }
  }, false);

  src.addEventListener(name, (event) => {
    handler(JSON.parse(event.data));
  });

  src.addEventListener("message", (event) => {
    console.log("Got generic message: " + event);
  });
}

function closeEventSources() {
  if (tracksStateStream != null) tracksStateStream.close();
  if (middleStateStream != null) middleStateStream.close();
  if (topStateStream != null) topStateStream.close();
  if (sttStream != null) sttStream.close();
  //if (headStateStream != null) headStateStream.close();
}

export default {
  data() {
    let arm_servos = {
        sh_left_fb: 10,
         sh_right_fb: 10,
         sh_left_ud: 27,
         sh_right_ud: 25,
         el_left: 43,
         el_right: 52,
         wr_left: 0,
         wr_right: -1,
         gr_left: 52,
         gr_right: -1
    };

    return {
      headlights: false,
      connected: null,
      track_state: {},
      middle_state: {},
      head_state: {},
      top_state: {},
      tts_input: "Hello my cute bunny",
      stt_output: "",
      voice_ready: true,
      arm_servos: arm_servos,
      neck_z_angle: 50,
      neck_y_angle: 50,
      lightring_val: "",
      lightring_vals: [],
    };
  },
  watch: {
    headlights: function (val) {
      axios
        .get(api + "/tracks/headlights/" + (val ? "on" : "off"))
        .then((response) => {
          console.log(response);
        })
        .catch((err) => {
          console.log(err);
        });
    },
    lightring_val: function (val) {
      axios
        .get(api + "/body/lightring?pat=" + val)
        .then((response) => {
          console.log(response);
        })
        .catch((err) => {
          console.log(err);
        });
    },
    arm_servos: {
      handler: function (val) {
        this.moveArms(val); 
      },
      deep: true
    },
    neck_z_angle: function (val) {
      console.log(val);
      this.moveNeck(); 
    },
    neck_y_angle: function (val) {
      console.log(val);
      this.moveNeck(); 
    },
    connected: function (val) {
      if(val) {
        this.connectToStreams();
        document.getElementById('vid-feed')
         .setAttribute('src', api + '/head/facefeed');
        document.getElementById('depth-feed')
         .setAttribute('src', api + '/body/depthfeed');

        joystick = nipplejs.create({
          zone: document.getElementById('joystick'),
          mode: 'static',
          position: {left: '50%', top: '50%'},
          color: 'red',
          shape: 'square',
          dynamicPage: true,
        });
        
        var that = this;

        joystick.on('start end', function(evt, data) {
          console.log(data);
        }).on('move', function(evt, data) {
          // Get x,y coordinates of the joystick in the -100,100 range
          var x = Math.round(data.vector.x * 100);
          var y = Math.round(data.vector.y * 100);
          that.controlTracks(x,y);
        });

        // Initila lightrring value
        this.lightring_val = "off";

      } else {
        this.closeStreams();
        document.getElementById('vid-feed')
         .setAttribute('src', '');
        document.getElementById('depth-feed')
         .setAttribute('src', '');
      }
    },
  },
  methods: {
    connect: function () {
      axios
        .get(api + "/connect")
        .then((res) => {
          this.connected = true;
          console.log("Connected: " + res.data);
        })
        .catch((err) => {
          console.log(err);
        });
    },
    disconnect: function () {
      axios
        .get(api + "/disconnect")
        .then((res) => {
          this.connected = false;
          console.log("Connected: " + res.data);
        })
        .catch((err) => {
          console.log(err);
        });
    },
    controlTracks: _.throttle( (x, y) => {
        console.log(x,y);
        axios
          .get(api + "/tracks/move", {params: {x: x, y:y}})
          .catch((err) => {
            console.log(err);
          });
    }, 300),
    headlightActionStr: function () {
      return this.headlights ? "off" : "on";
    },
    speak: function () {
      if (!this.tts_input) return;
      this.voice_ready = false;
      axios
        .get(api + "/voice/speak", {params: { tts: this.tts_input }})
        .then((res) => {
          console.log(res);
          this.voice_ready = true;
        })
        .catch((err) => {
          console.log(err);
          this.voice_ready = true;
        });
    },
    triggerFan: function () {
      axios
        .get(api + "/body/fan/on")
        .then((res) => {
          console.log(res);
        })
        .catch((err) => {
          console.log(err);
        });
    },
    moveArms: _.throttle( (vals) => {
       // Ignore -1 values
       let filtered = Object.entries(vals).filter( x => x[1] > 0 );
       const params = new URLSearchParams(filtered).toString(); 
       axios
        .get(api + "/body/move_arms?" + params)
        .then((res) => {
          console.log(res);
        })
        .catch((err) => {
          console.log(err);
        });
    }, 100),
    moveNeck: function() {
       axios
        .get(api + "/body/move_neck?z=" + this.neck_z_angle + "&y=" + this.neck_y_angle)
        .then((res) => {
          console.log(res);
        })
        .catch((err) => {
          console.log(err);
        });
    },
    connectToStreams: function(){
      tracksStateStream = new EventSource(api + "/tracks/stream");
      setupEventSource(tracksStateStream, "tracks", (data) => this.track_state = data);

      middleStateStream = new EventSource(api + "/middle/stream");
      setupEventSource(middleStateStream, "middle", (data) => this.middle_state = data);

      topStateStream = new EventSource(api + "/top/stream");
      setupEventSource(topStateStream, "top", (data) => this.top_state = data);

      sttStream = new EventSource(api + "/voice/stream");
      setupEventSource(sttStream, "voice", (data) => {
        this.stt_output = this.stt_output.concat(data.transcript + "\n")
        // Always keep scrolling to the bottom of the stt output
        var ta = document.getElementById('textarea');
        ta.scrollTop = ta.scrollHeight;
      });

      // headStateStream = new EventSource(api + "/head/stream");
      // setupEventSource(headStateStream, "head", (data) => this.head_state = data);
    },
    closeStreams: function() {
      closeEventSources();
    },
  },
  mounted() {
    const vals = ["off", "red", "blue", "green", "rainbow"];
    this.lightring_vals = vals.map(x => ({value: x, text: x}));

    axios
      .get(api + "/")
      .then((response) => {
        this.connected = response.data.connected;
      })
      .catch((err) => {
        console.log(err);
      });

    axios
      .get(api + "/tracks/headlights")
      .then((response) => {
        this.headlights = response.data.value == "True" ? true : false;
      })
      .catch((err) => {
        console.log(err);
      });
  },
};
</script>
