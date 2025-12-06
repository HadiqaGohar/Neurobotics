const webpack = require('webpack');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-webpack-env',
    configureWebpack(config, isServer) {
      return {
        plugins: [
          new webpack.DefinePlugin({
            'process.env.NODE_ENV': JSON.stringify(process.env.NODE_ENV || 'development'),
            'process.env.REACT_APP_API_URL': JSON.stringify(process.env.REACT_APP_API_URL || 'http://localhost:3000/api'),
            // Add other environment variables here as needed
          }),
        ],
      };
    },
  };
};