const canvas = document.getElementById('mujocoCanvas');
const ctx = canvas.getContext('2d');

let lastFrameId = 0;

const ws = new WebSocket('ws://localhost:9002');
ws.binaryType = 'blob';

ws.onopen = () => console.log("[WS] Connected to MuJoCo");
ws.onclose = () => console.warn("[WS] Disconnected");

ws.onmessage = (event) => {
    const reader = new FileReader();
    reader.onload = function () {
        const buffer = reader.result;
        const view = new DataView(buffer);
        const frameId = view.getUint32(0, false);  // Big endian (network byte order)
        //console log every frame id
        console.log("Frame ID:", frameId);
        if (frameId < lastFrameId) {
            // Discard immediately
            // console log fram id

            return;
        }

        const blob = new Blob([buffer.slice(4)], { type: 'image/jpeg' });
        const url = URL.createObjectURL(blob);

        const img = new Image();
        img.onload = () => {
            // Onload can happen much later — verify again
            if (frameId < lastFrameId) {
                URL.revokeObjectURL(url);
                return;
            }

            lastFrameId = frameId;
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            URL.revokeObjectURL(url);
        };
        img.src = url;
    };
    reader.readAsArrayBuffer(event.data);
};


// Mouse interaction handling
let lastMouse = { x: 0, y: 0 };
let buttons = { left: false, middle: false, right: false };

canvas.addEventListener('mousedown', (e) => {
    buttons.left = e.button === 0;
    buttons.middle = e.button === 1;
    buttons.right = e.button === 2;

    ws.send(JSON.stringify({
        type: 'mouse_button',
        left: buttons.left,
        middle: buttons.middle,
        right: buttons.right
    }));

    lastMouse.x = e.clientX;
    lastMouse.y = e.clientY;
});

canvas.addEventListener('mouseup', () => {
    buttons = { left: false, middle: false, right: false };
    ws.send(JSON.stringify({ type: 'mouse_button', ...buttons }));
});

canvas.addEventListener('mousemove', (e) => {
    const dx = e.clientX - lastMouse.x;
    const dy = e.clientY - lastMouse.y;
    lastMouse.x = e.clientX;
    lastMouse.y = e.clientY;

    if (buttons.left || buttons.middle || buttons.right) {
        ws.send(JSON.stringify({ type: 'mouse_move', dx, dy }));
    }
});

canvas.addEventListener('wheel', (e) => {
    ws.send(JSON.stringify({ type: 'scroll', dy: e.deltaY }));
    e.preventDefault();  // prevent page scroll
});
