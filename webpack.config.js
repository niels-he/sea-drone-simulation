const path = require('path');
const webpack = require('webpack');

module.exports = {
    entry: './src/index.js',
    mode:'development',
    output: {
        path: path.resolve(__dirname, 'dist'),
        filename: 'sea-drone.js',
        library: 'sea-drone'
        //libraryTarget: 'umd'
    },
    devtool: false,
    plugins: [
        new webpack.SourceMapDevToolPlugin({})
    ]
};
