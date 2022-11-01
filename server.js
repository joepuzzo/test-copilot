// express server on port 4000
const express = require('express'); 

const app = express();  

// return the current date and time
app.get('/date', (req, res) => {
    res.send(new Date().toString());
} );  


// start the server on port 4000  
app.listen(4000, () => console.log('Server started on port 4000'));

// inverse matrix
// Path: server.js