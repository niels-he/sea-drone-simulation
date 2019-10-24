const path = require('path');

module.exports = {
    entry: './src/index.js',
    mode:'development',
    output: {
        path: path.resolve(__dirname, 'dist'),
        filename: 'sea-drone.js',
        library: 'sea-drone'
        //libraryTarget: 'umd'
    },
};
