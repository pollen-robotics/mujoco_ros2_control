const canvas = document.getElementById('mujocoCanvas');
const ctx = canvas.getContext('2d');
const img = new Image();

const ws = new WebSocket('ws://localhost:9002');  // C++ WebSocket server
ws.binaryType = 'blob';

ws.onmessage = (event) => {
    const blob = event.data;
    const url = URL.createObjectURL(blob);
    img.onload = () => {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        URL.revokeObjectURL(url);
    };
    img.src = url;
};

ws.onopen = () => console.log("[WS] Connected to MuJoCo");
ws.onclose = () => console.warn("[WS] Disconnected");
