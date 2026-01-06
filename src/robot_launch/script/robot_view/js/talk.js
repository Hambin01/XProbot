const startBtn = document.getElementById('talk');
let mediaStream = null;
let audioContext = null;
let websocket = null;
let processor = null;
const sampleRate = 44100;
const chunkSize = 2048;
let audioPlayer = document.getElementById('audioPlayer');
const channels = 1;
start_flag = false;

startBtn.addEventListener('click', async () => {
    start_flag = !start_flag;
    if (start_flag) {
        startBtn.className = 'icon-button button-talk-on';
        if (!navigator.mediaDevices) {
            alert('getUserMedia not supported on your browser!');
            return;
        }

        try {
            console.log('start talk');
            mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
            audioContext = new (window.AudioContext || window.webkitAudioContext)({ sampleRate });
            const source = audioContext.createMediaStreamSource(mediaStream);
            processor = audioContext.createScriptProcessor(chunkSize, 1, 1);
            source.connect(processor);
            processor.connect(audioContext.destination);

            websocket = new WebSocket('ws://10.168.4.210:8765');
            websocket.binaryType = 'arraybuffer';

            processor.onaudioprocess = (event) => {
                const inputData = event.inputBuffer.getChannelData(0);
                const buffer = new ArrayBuffer(inputData.length * 2);
                const view = new DataView(buffer);

                for (let i = 0; i < inputData.length; i++) {
                    view.setInt16(i * 2, inputData[i] * 0x7FFF, true);
                }

                if (websocket && websocket.readyState === WebSocket.OPEN) {
                    websocket.send(buffer);
                }
            };

            websocket.onmessage = (event) => {
                const arrayBuffer = event.data;
                const uint16Array = new Uint16Array(arrayBuffer);
                const audioBlob = convertUint16ArrayToWavBlob(uint16Array);
                const audioUrl = URL.createObjectURL(audioBlob);

                // Ensure audioUrl is correctly generated
                console.log('Generated audioUrl:', audioUrl);

                audioPlayer.src = audioUrl;
                audioPlayer.playbackRate = 0.5;
                audioPlayer.play().catch(function (error) {
                    console.error('Error playing audio:', error);
                });
            };


        } catch (err) {
            console.error('Error accessing audio stream:', err);
        }
    } else {
        startBtn.className = 'icon-button button-talk';
        console.log('stop talk');
        if (mediaStream) {
            mediaStream.getTracks().forEach(track => track.stop());
        }
        if (processor) {
            processor.disconnect();
        }
        if (audioContext) {
            audioContext.close();
        }
        if (websocket) {
            websocket.close();
        }
    }
});


function convertUint16ArrayToWavBlob(uint16Array) {
    const buffer = new ArrayBuffer(44 + uint16Array.length * 2);
    const view = new DataView(buffer);

    /* RIFF identifier */
    writeString(view, 0, 'RIFF');
    /* file length */
    view.setUint32(4, 36 + uint16Array.length * 2, true);
    /* RIFF type */
    writeString(view, 8, 'WAVE');
    /* format chunk identifier */
    writeString(view, 12, 'fmt ');
    /* format chunk length */
    view.setUint32(16, 16, true);
    /* sample format (raw) */
    view.setUint16(20, 1, true);
    /* channel count */
    view.setUint16(22, channels, true);
    /* sample rate */
    view.setUint32(24, sampleRate, true);
    /* byte rate (sample rate * block align) */
    view.setUint32(28, sampleRate * channels * 2, true);
    /* block align (channel count * bytes per sample) */
    view.setUint16(32, channels * 2, true);
    /* bits per sample */
    view.setUint16(34, 16, true);
    /* data chunk identifier */
    writeString(view, 36, 'data');
    /* data chunk length */
    view.setUint32(40, uint16Array.length * 2, true);

    for (let i = 0; i < uint16Array.length; i++) {
        view.setInt16(44 + i * 2, uint16Array[i], true);
    }

    return new Blob([buffer], { type: 'audio/wav' });
}

function writeString(view, offset, string) {
    for (let i = 0; i < string.length; i++) {
        view.setUint8(offset + i, string.charCodeAt(i));
    }
}
